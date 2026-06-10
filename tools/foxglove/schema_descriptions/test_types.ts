// Test type aliases used to verify the Foxglove schema codegen tool.
//
// These are intentionally exhaustive over the subset of TypeScript that the generator supports.
// If you change these types, regenerate the golden file by running:
//
//   bazel run @aos//tools/foxglove/schema_descriptions:copy_generated_file
//
// and re-run the comparison test:
//
//   bazel test @aos//tools/foxglove/schema_descriptions:compare_generated_file

// Covers all primitive scalar types.
export type SimplePrimitives = {
  name: string;
  count: number;
  active: boolean;
  raw_data: Uint8Array;
};

// Covers arrays of primitives and optional fields (`T | undefined`).
export type ArraysAndOptional = {
  labels: string[];
  weights?: number[];
  flags: boolean[];
};

// Covers nested object types and arrays of objects.
export type Nested = {
  position: {
    x: number;
    y: number;
  };
  waypoints: {
    id: string;
    time: number;
  }[];
};
