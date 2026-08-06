// Code generator for Foxglove `MessageSchemaDescription` literals.
//
// Given a TypeScript source file and a list of type alias names, this tool emits a
// `.generated.ts` file containing one `MessageSchemaDescription` constant per type whose
// structure mirrors the input type. The intent is to let topic converters keep their TypeScript
// output type as the single source of truth and avoid hand-duplicating the same shape as a
// `schemaDescription`.
//
// The goal is to use the generated output in the `outputSchemaDescription` field of a topic
// converter. See the upstream documentation for more information.
// https://docs.foxglove.dev/docs/extensions/guides/create-topic-converter
//
// Usage:
//   node generate_foxglove_schema_description.js <input.ts> <output.generated.ts> <TypeName>...
//
// Under Bazel, use the `foxglove_schema_descriptions` macro:
//   load(
//     "@aos//tools/foxglove/schema_descriptions:foxglove_schema.bzl,
//     "foxglove_schema_descriptions",
//   )
//
// Only a small subset of TypeScript is supported — enough to express simple flat message shapes.
// Anything more exotic intentionally throws so the failure is loud rather than silent.
//
// Supported:
//   * `number`         -> "number"
//   * `string`         -> "string"
//   * `boolean`        -> "bool"
//   * `Uint8Array`     -> ["byte"]
//   * `T[]`            -> [<schema for T>]
//   * Inline object types (type literals)
//   * Optional fields (the `| undefined` is stripped)
//
// Explicitly unsupported (would need more work):
//   * Type references to other named types (the type literal must be inlined)
//   * Union types other than `T | undefined`
//   * `bigint` (Foxglove has no equivalent; pick a representation deliberately if needed)
//   * Enums

import * as fs from 'node:fs';
import * as path from 'node:path';
import * as ts from 'typescript';

// Intermediate, language-agnostic representation of a Foxglove schema:
//   * a primitive name (e.g. "number", "string", "bool", "byte"),
//   * a one-element array whose sole entry is the element schema, or
//   * a plain object mapping property names to nested schemas.
// `renderSchema` converts a value of this type into TypeScript source.
type Schema = string | [Schema] | {[key: string]: Schema};

// Print a prefixed error message to stderr and terminate the process with a non-zero exit code.
// Used for user-facing failures (bad CLI arguments, missing type aliases, unsupported constructs)
// where a stack trace would be noise.
function fail(message: string): never {
  process.stderr.write(`generate_foxglove_schema_description: ${message}\n`);
  process.exit(1);
}

// Walk `sourceFile` looking for a top-level (or nested) `type <typeName> = ...` declaration and
// return its AST node, or `undefined` if no such alias exists. Only the first match is returned;
// callers are expected to use unique type names.
function findTypeAlias(
  sourceFile: ts.SourceFile,
  typeName: string
): ts.TypeAliasDeclaration | undefined {
  let found: ts.TypeAliasDeclaration | undefined;

  function visit(node: ts.Node): void {
    if (found) return;
    if (ts.isTypeAliasDeclaration(node) && node.name.text === typeName) {
      found = node;
      return;
    }
    ts.forEachChild(node, visit);
  }

  visit(sourceFile);

  return found;
}

// Strip `| undefined` from a union, returning the remaining single type node. This is how we
// model optional fields: `foo?: T` and `foo: T | undefined` both reduce to the schema for `T`.
// Any other union (e.g. `string | number`) is rejected because Foxglove schemas have no concept
// of sum types.
function stripUndefined(typeNode: ts.TypeNode): ts.TypeNode {
  if (!ts.isUnionTypeNode(typeNode)) return typeNode;
  const remaining = typeNode.types.filter(
    (t) => t.kind !== ts.SyntaxKind.UndefinedKeyword
  );
  if (remaining.length === 1) return remaining[0];
  throw new Error(`Unsupported union type: ${typeNode.getText()}`);
}

// Recursively convert a TypeScript type AST node into the in-memory schema representation
// (see the `Schema` type above). Throws if it encounters a TypeScript construct outside the
// supported subset documented at the top of this file.
function typeNodeToSchema(typeNode: ts.TypeNode): Schema {
  const node = stripUndefined(typeNode);

  switch (node.kind) {
    case ts.SyntaxKind.NumberKeyword:
      return 'number';
    case ts.SyntaxKind.StringKeyword:
      return 'string';
    case ts.SyntaxKind.BooleanKeyword:
      return 'bool';
  }

  if (ts.isArrayTypeNode(node)) {
    return [typeNodeToSchema(node.elementType)];
  }

  if (ts.isTypeLiteralNode(node)) {
    const result: {[key: string]: Schema} = {};
    for (const member of node.members) {
      if (!ts.isPropertySignature(member) || !member.type) {
        throw new Error(`Unsupported member: ${member.getText()}`);
      }
      if (!ts.isIdentifier(member.name) && !ts.isStringLiteral(member.name)) {
        throw new Error(`Unsupported property name: ${member.name.getText()}`);
      }
      const propertyName = member.name.text;
      result[propertyName] = typeNodeToSchema(member.type);
    }
    return result;
  }

  if (ts.isTypeReferenceNode(node)) {
    const referenceName = node.typeName.getText();
    if (referenceName === 'Uint8Array') return ['byte'];
    throw new Error(
      `Unsupported type reference '${referenceName}'. Only inline type literals are supported in ` +
        `this prototype.`
    );
  }

  throw new Error(
    `Unsupported type node: ${ts.SyntaxKind[node.kind]} (${node.getText()})`
  );
}

// Pretty-print an in-memory schema (as produced by `typeNodeToSchema`) as a TypeScript object
// literal string. `indent` is the current nesting level measured in tab stops and is used to
// align nested braces with their parent. The output is intended to be embedded directly into
// a `const ... = ...;` declaration in the generated file.
function renderSchema(schema: Schema, indent: number): string {
  const pad = '\t'.repeat(indent);
  const inner = '\t'.repeat(indent + 1);
  if (Array.isArray(schema)) {
    if (schema.length !== 1)
      throw new Error('Array schema must have exactly one element');
    return `[${renderSchema(schema[0], indent)}]`;
  }
  if (typeof schema === 'string') {
    return JSON.stringify(schema);
  }
  const entries = Object.entries(schema);
  if (entries.length === 0) return '{}';
  const lines = entries.map(
    ([key, value]) => `${inner}${key}: ${renderSchema(value, indent + 1)},`
  );
  return `{\n${lines.join('\n')}\n${pad}}`;
}

// Entry point. Parses CLI arguments, loads the input TypeScript file, looks up each requested
// type alias, converts it to a schema, and writes a single `.generated.ts` file exporting one
// `<TypeName>SchemaDescription` constant per type. Fails loudly via `fail()` if any step is
// invalid so Bazel surfaces a clear error instead of producing a half-baked output.
function main(): void {
  const [, , inputPath, outputPath, ...typeNames] = process.argv;
  if (!inputPath || !outputPath || typeNames.length === 0) {
    fail('expected arguments: <input.ts> <output.generated.ts> <TypeName>...');
  }

  const sourceText = fs.readFileSync(inputPath, 'utf8');
  const sourceFile = ts.createSourceFile(
    path.basename(inputPath),
    sourceText,
    ts.ScriptTarget.Latest,
    /*setParentNodes=*/ true
  );

  const exports: string[] = [];
  for (const typeName of typeNames) {
    const typeAlias = findTypeAlias(sourceFile, typeName);
    if (!typeAlias)
      fail(`could not find type alias '${typeName}' in ${inputPath}`);

    let schema: Schema;
    try {
      schema = typeNodeToSchema(typeAlias.type);
    } catch (error) {
      fail(
        `while generating schema for '${typeName}': ${(error as Error).message}`
      );
    }
    exports.push(
      `export const ${typeName}SchemaDescription: MessageSchemaDescription = ${renderSchema(
        schema,
        0
      )};`
    );
  }

  const output =
    `// AUTO-GENERATED FILE — do not edit by hand.\n` +
    `// Generated from ${path.basename(
      inputPath
    )} by tools/foxglove/schema_descriptions.\n` +
    `// Regenerate by rebuilding the surrounding Bazel target.\n` +
    `\n` +
    `import { MessageSchemaDescription } from '@foxglove/extension';\n` +
    `\n` +
    exports.join('\n\n') +
    `\n`;

  // Under `js_run_binary`, the working directory is `<execroot>/$BAZEL_BINDIR`, but
  // `$(execpath ...)` substitutes paths relative to the execroot, including the bindir prefix.
  // Strip that prefix so the file lands in the expected output location instead of a
  // double-nested `bazel-out/.../bin/bazel-out/.../bin/...` path.
  let resolvedOutput = outputPath;
  const bindir = process.env.BAZEL_BINDIR;
  if (bindir && resolvedOutput.startsWith(bindir + '/')) {
    resolvedOutput = resolvedOutput.slice(bindir.length + 1);
  }
  fs.mkdirSync(path.dirname(resolvedOutput), {recursive: true});
  fs.writeFileSync(resolvedOutput, output);
}

main();
