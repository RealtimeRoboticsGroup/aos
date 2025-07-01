#include "aos/flatbuffers.h"

#include "absl/log/log.h"
#include "absl/strings/str_cat.h"
#include "gtest/gtest.h"

#include "aos/json_to_flatbuffer.h"
#include "aos/json_to_flatbuffer_generated.h"
#include "aos/realtime.h"
#include "aos/testing/tmpdir.h"

namespace aos::testing {

// Tests that Verify works.
TEST(FlatbufferTest, Verify) {
  FlatbufferDetachedBuffer<Configuration> fb =
      JsonToFlatbuffer<Configuration>("{}");
  FlatbufferSpan<Configuration> fb_span(fb);
  EXPECT_TRUE(fb.Verify());
  EXPECT_TRUE(fb_span.Verify());

  // Now confirm it works on an empty flatbuffer.
  FlatbufferSpan<Configuration> empty(absl::Span<const uint8_t>(nullptr, 0));
  EXPECT_FALSE(empty.Verify());
}

// Test that the UnpackFlatbuffer builds & works.
TEST(FlatbufferTest, UnpackFlatbuffer) {
  const FlatbufferDetachedBuffer<Location> fb =
      JsonToFlatbuffer<Location>("{\"name\": \"abc\", \"frequency\": 118}");

  LocationT object = UnpackFlatbuffer(&fb.message());
  EXPECT_EQ("abc", object.name);
  EXPECT_EQ(118, object.frequency);
}

// Tests the ability to map a flatbuffer on disk to memory
TEST(FlatbufferMMapTest, Verify) {
  FlatbufferDetachedBuffer<Configuration> fb =
      JsonToFlatbuffer<Configuration>("{\"foo_int\": 3}");

  const std::string fb_path = absl::StrCat(TestTmpDir(), "/fb.bfbs");
  WriteFlatbufferToFile(fb_path, fb);

  FlatbufferMMap<Configuration> fb_mmap(fb_path);
  EXPECT_TRUE(fb.Verify());
  EXPECT_TRUE(fb_mmap.Verify());
  ASSERT_EQ(fb_mmap.message().foo_int(), 3);

  // Verify that copying works
  {
    FlatbufferMMap<Configuration> fb_mmap2(fb_path);
    fb_mmap2 = fb_mmap;
    EXPECT_TRUE(fb_mmap.Verify());
    EXPECT_TRUE(fb_mmap2.Verify());
    ASSERT_EQ(fb_mmap2.message().foo_int(), 3);
    ASSERT_EQ(fb_mmap.message().foo_int(), 3);
  }
  EXPECT_TRUE(fb_mmap.Verify());
  ASSERT_EQ(fb_mmap.message().foo_int(), 3);

  // Verify that moving works
  {
    FlatbufferMMap<Configuration> fb_mmap3(fb_path);
    fb_mmap3 = std::move(fb_mmap);
    EXPECT_TRUE(fb_mmap3.Verify());
    ASSERT_EQ(fb_mmap3.message().foo_int(), 3);
  }
}

// Tests the ability to modify a flatbuffer mmaped from on disk in memory
TEST(FlatbufferMMapTest, Writeable) {
  FlatbufferDetachedBuffer<Configuration> fb =
      JsonToFlatbuffer<Configuration>("{\"foo_int\": 3}");

  const std::string fb_path = absl::StrCat(TestTmpDir(), "/fb.bfbs");
  WriteFlatbufferToFile(fb_path, fb);

  {
    FlatbufferMMap<Configuration> fb_mmap(fb_path,
                                          util::FileOptions::kWriteable);
    fb_mmap.mutable_message()->mutate_foo_int(5);
  }

  {
    FlatbufferMMap<Configuration> fb_mmap(fb_path);
    EXPECT_EQ(fb_mmap.message().foo_int(), 5);
  }
}

// Validates that we can successfully instantiate and use a
// FlatbufferFixedAllocatorArray in realtime code.
TEST(FlatbufferFixedAllocatorArrayTest, UseInRealtime) {
  aos::ScopedRealtime realtime;

  FlatbufferFixedAllocatorArray<Configuration, 1000> allocator_array;

  // Construct the message with arbitrary contents.
  {
    Configuration::Builder builder(*allocator_array.fbb());
    builder.add_foo_int(1);
    allocator_array.Finish(builder.Finish());

    // Get a pointer to it and validate it's what we expect.
    const Configuration *config = &allocator_array.message();
    ASSERT_TRUE(config->has_foo_int());
    EXPECT_EQ(config->foo_int(), 1);
  }

  // Perform a reset so we can rebuild the message.
  allocator_array.Reset();

  // Now construct the message slightly differently.
  {
    Configuration::Builder builder(*allocator_array.fbb());
    builder.add_foo_int(2);
    allocator_array.Finish(builder.Finish());

    // Get a pointer to the new message and validate its contents.
    const Configuration *config = &allocator_array.message();
    ASSERT_TRUE(config->has_foo_int());
    EXPECT_EQ(config->foo_int(), 2);
  }
}

#if __has_feature(memory_sanitizer) || __has_feature(address_sanitizer)

// Validates that we can detect bugs similar to use-after-free when using a
// FlatbufferFixedAllocatorArray.
TEST(FlatbufferFixedAllocatorArrayDeathTest, DetectsUseAfterReset) {
  FlatbufferFixedAllocatorArray<Configuration, 1000> allocator_array;

  // Construct the message with arbitrary contents initially.
  {
    Configuration::Builder builder(*allocator_array.fbb());
    builder.add_foo_int(1);
    allocator_array.Finish(builder.Finish());
  }

  // Get a pointer to it and validate it's what we expect.
  const Configuration *config1 = &allocator_array.message();
  ASSERT_TRUE(config1->has_foo_int());
  EXPECT_EQ(config1->foo_int(), 1);

  // Perform the reset that should trigger an error.
  allocator_array.Reset();

  // Now construct the message slightly differently.
  {
    Configuration::Builder builder(*allocator_array.fbb());
    builder.add_foo_int(2);
    allocator_array.Finish(builder.Finish());
  }

  // Get a pointer to the new message and validate its contents.
  const Configuration *config2 = &allocator_array.message();
  ASSERT_TRUE(config2->has_foo_int());
  EXPECT_EQ(config2->foo_int(), 2);

  // Now accessing the old message we initially constructed should result in a
  // failure.
  EXPECT_DEATH(
      { ABSL_LOG(INFO) << "config1->foo_int() = " << config1->foo_int(); },
#if __has_feature(memory_sanitizer)
      "use-of-uninitialized-value"
#else
      "heap-use-after-free"
#endif
  );
}

#endif

}  // namespace aos::testing
