// This script acts as a wrapper for the `foxglove-extension` binary. We need a
// wrapper here because `foxglove-extension` wants to invoke `npm` directly.
// Since we don't support the real npm binary, we force `foxglove-extension` to
// use our fake npm binary (`foxglove_extension_wrapper_npm.js`) by
// manipulating the PATH.

const {spawnSync} = require("child_process");
const path = require("path");
const process = require("process");
const fs = require("fs");
const {tmpdir} = require("os");

// Add a directory to the PATH environment variable.
function addToPath(directory) {
  const currentPath = process.env.PATH || "";
  const newPath = `${directory}${path.delimiter}${currentPath}`;
  process.env.PATH = newPath;
}

const isWin = process.platform === "win32";
const fakeNpm = path.join(
  __dirname,
  isWin
    ? "foxglove_extension_wrapper_npm_/foxglove_extension_wrapper_npm.bat"
    : "foxglove_extension_wrapper_npm_/foxglove_extension_wrapper_npm"
);

const tempBinDir = fs.mkdtempSync(
  path.join(tmpdir(), "foxglove_extension_wrapper-tmp-")
);

if (isWin) {
  fs.writeFileSync(
    path.join(tempBinDir, "npm.bat"),
    `@echo off\r\ncall "${fakeNpm}" %*\r\n`
  );
} else {
  fs.symlinkSync(fakeNpm, path.join(tempBinDir, "npm"));
}

addToPath(tempBinDir);

// Create a relative path for a specific root-relative directory.
function getRelativePath(filePath) {
  // Count the number of directories and construct the relative path.
  const numDirectories = filePath.split("/").length;
  return "../".repeat(numDirectories);
}

bazel_package = process.env.BAZEL_REPOSITORY + process.env.BAZEL_PACKAGE;

// We need to know the path to the `foxglove-extension` binary from the
// sub-directory where we're generating code into.
const relativePath = getRelativePath(bazel_package);
let foxgloveExtensionPath = path.join(
  relativePath,
  process.env.BAZEL_REPOSITORY +
    `tools/foxglove/foxglove_extension_/foxglove_extension`
);
if (isWin) {
  foxgloveExtensionPath += ".bat";
}

// Extract arguments intended for the `foxglove-extension` binary.
const args = process.argv.slice(2);

// Execute the `foxglove-extension` binary.
try {
  const result = spawnSync(foxgloveExtensionPath, args, {
    stdio: "inherit",
    cwd: bazel_package,
    shell: isWin,
  });
  if (result.error) {
    console.error("Error executing foxglove_extension:", result.error);
    process.exit(1);
  }
  if (result.status !== 0) {
    console.error(`foxglove_extension exited with status ${result.status}`);
    process.exit(result.status);
  }
} catch (error) {
  console.error("Error executing foxglove_extension:", error);
  process.exit(1);
}
