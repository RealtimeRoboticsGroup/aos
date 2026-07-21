# AOS patches for flatbuffers

These patches are applied via `single_version_override` in the root
`MODULE.bazel` to the [BCR-distributed flatbuffers module](https://registry.bazel.build/modules/flatbuffers).

The intent is that the AOS-specific divergence from upstream flatbuffers lives
here as a curated list of small logical patches.

## Sequencing

The numeric prefixes determine the order in which patches are applied. The
ordering is significant because some patches modify the same file; therefore,
changes must be sequenced cleanly.

## Reflection schema handling

AOS uses the reflection schema for introspection, the static generator,
MCAP/Foxglove logging, binary annotation, and test infrastructure. Therefore it
needs generated features like the object API, which the upstream version doesn't
have.

Note that we also add IDs to the fields of the schema because AOS uses the
`--require-explicit-ids` flag with `flatc`, and compilation would fail when this
file gets included.

Patch 17 is the single source of truth for the schema changes *and* the full
AOS-flavored `reflection_generated.h`.

Note: The local copy of `reflection.fbs` (in
`aos/flatbuffers/reflection/BUILD.bazel`) is retained only for TS/Rust bindings
and static generation test infrastructure. No header shadowing or extra
AOS-owned Bazel targets are used for the C++ header.

### Regenerating the reflection parts of patch 17

When upgrading the flatbuffers version, follow these steps to update the
reflection-related hunks in patch 17:

1. Check out a clean tree of the *target* flatbuffers release (the one you
   are upgrading to).

2. Locate the original (unpatched) `reflection/reflection.fbs` in that tree.

3. Apply the schema changes from the current patch 17's `reflection/reflection.fbs`
   hunk (or manually add `(id: 0)`, `(id: 1)`, ... to every field in every
   table/enum in the order they appear, plus any other schema tweaks that
   have accumulated in patch 17).

4. Build (or acquire) the `flatc` binary for the target flatbuffers version.

5. Run `flatc` with *exactly* the flags from AOS's `DEFAULT_FLATC_ARGS`
   (see `aos/flatbuffers/build_defs.bzl`). This produces
   `/tmp/out/reflection_generated.h`.

6. In the *current* AOS tree:
   - Replace the `include/flatbuffers/reflection_generated.h` section of patch
     17 with a fresh `diff --git a/include/flatbuffers/reflection_generated.h`
     hunk against the original header from the target flatbuffers tree, using
     the newly generated file as the "new" content.
   - Verify that the other hunks in patch 17 (minireflect.h, reflection.h,
     generators, binary_annotator, etc.) still apply cleanly to the new
     upstream sources. Rebase/adjust as needed.

7. Update the top-level description of patch 17 if the set of changes has
   evolved.

8. Test that the module remains buildable and that AOS reflection users work:
   ```
   bazel test -c opt --enable_bzlmod --noenable_workspace \
     //aos/flatbuffers/... \
     //aos/util:mcap_logger_test \
     //aos/events/logging:logfile_utils_test
   ```
   (Also run the equivalent in workspace mode.)

9. Commit the updated patch 17 (and any other adjusted patches) with a clear
   message referencing the flatbuffers upgrade.
