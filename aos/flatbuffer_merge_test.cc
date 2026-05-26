#include "aos/flatbuffer_merge.h"

#include <cmath>
#include <limits>
#include <string_view>

#include "absl/log/absl_log.h"
#include "absl/strings/escaping.h"
#include "flatbuffers/minireflect.h"
#include "gtest/gtest.h"

#include "aos/json_to_flatbuffer.h"
#include "aos/json_to_flatbuffer_generated.h"
#include "aos/realtime.h"
#include "aos/sanitizers.h"

namespace aos::testing {

std::string_view FromFbb(const flatbuffers::FlatBufferBuilder &fbb) {
  return std::string_view(
      reinterpret_cast<const char *>(fbb.GetCurrentBufferPointer()),
      fbb.GetSize());
}

std::string_view FromFbb(const FlatbufferDetachedBuffer<Configuration> &b) {
  return std::string_view(reinterpret_cast<const char *>(b.span().data()),
                          b.span().size());
}

class FlatbufferMerge : public ::testing::Test {
 public:
  FlatbufferMerge() {}

  void ExpectMergedOutput(
      const NonSizePrefixedFlatbuffer<Configuration> &fb_merged,
      std::string_view expected_output) {
    ASSERT_NE(fb_merged.span().size(), 0u);

    const ::std::string merged_output = FlatbufferToJson(fb_merged);
    EXPECT_EQ(expected_output, merged_output);

    aos::FlatbufferDetachedBuffer<Configuration> expected_message(
        JsonToFlatbuffer(std::string(expected_output).c_str(),
                         ConfigurationTypeTable()));
    EXPECT_TRUE(
        CompareFlatBuffer(&fb_merged.message(), &expected_message.message()));
  }

  void JsonMerge(const ::std::string in1, const ::std::string in2,
                 const ::std::string out) {
    ABSL_LOG(INFO) << "Merging: " << in1.c_str();
    ABSL_LOG(INFO) << "Merging: " << in2.c_str();
    const FlatbufferDetachedBuffer<Configuration> fb1 = JsonToFlatbuffer(
        static_cast<const char *>(in1.c_str()), ConfigurationTypeTable());

    const ::std::string in1_nested = "{ \"nested_config\": " + in1 + " }";
    const FlatbufferDetachedBuffer<Configuration> fb1_nested =
        JsonToFlatbuffer(static_cast<const char *>(in1_nested.c_str()),
                         ConfigurationTypeTable());

    const FlatbufferDetachedBuffer<Configuration> fb2 = JsonToFlatbuffer(
        static_cast<const char *>(in2.c_str()), ConfigurationTypeTable());

    const ::std::string in2_nested = "{ \"nested_config\": " + in2 + " }";
    const FlatbufferDetachedBuffer<Configuration> fb2_nested =
        JsonToFlatbuffer(static_cast<const char *>(in2_nested.c_str()),
                         ConfigurationTypeTable());

    const ::std::string out_nested = "{ \"nested_config\": " + out + " }";

    const FlatbufferDetachedBuffer<Configuration> empty =
        JsonToFlatbuffer("{ }", ConfigurationTypeTable());

    ASSERT_NE(fb1.span().size(), 0u);
    ASSERT_NE(fb2.span().size(), 0u);
    ASSERT_NE(fb1_nested.span().size(), 0u);
    ASSERT_NE(fb2_nested.span().size(), 0u);

    // We now want to run 7 tests.
    //  in1 merged "" -> in1.
    //  in2 merged "" -> in2.
    //  "" merged in1 -> in1.
    //  "" merged in2 -> in2.
    //  nullptr merged in1 -> in1.
    //  in1 merged nullptr -> in1.
    //  in1 merged in2 -> out.

    {
      // in1 merged with "" => in1.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(fb1, empty);
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in1, FlatbufferToJson(fb_merged));
    }

    {
      // in2 merged with "" => in2.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(fb2, empty);
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in2, FlatbufferToJson(fb_merged));
    }

    {
      // "" merged with in1 => in1.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(empty, fb1);
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in1, FlatbufferToJson(fb_merged));
    }

    {
      // "" merged with in2 => in2.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(empty, fb2);
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in2, FlatbufferToJson(fb_merged));
    }

    {
      // nullptr merged with in1 => in1.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(nullptr, &fb1.message());
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in1, FlatbufferToJson(fb_merged));
    }

    {
      // in1 merged with nullptr => in1.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(&fb1.message(), nullptr);
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in1, FlatbufferToJson(fb_merged));
    }

    {
      // in1 merged with in2 => out.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(fb1, fb2);

      ExpectMergedOutput(fb_merged, out);
    }

    // Test all the different merge methods:
    ExpectMergedOutput(MergeFlatBuffers<Configuration>(fb1, fb2), out);
    {
      flatbuffers::FlatBufferBuilder fbb;
      fbb.ForceDefaults(true);
      fbb.Finish(MergeFlatBuffers(
          ConfigurationTypeTable(),
          reinterpret_cast<const flatbuffers::Table *>(&fb1.message()),
          reinterpret_cast<const flatbuffers::Table *>(&fb2.message()), &fbb));
      FlatbufferDetachedBuffer<Configuration> fb(fbb.Release());
      ExpectMergedOutput(fb, out);
    }
    {
      flatbuffers::FlatBufferBuilder fbb;
      fbb.ForceDefaults(true);
      fbb.Finish(MergeFlatBuffers<Configuration>(&fb1.message(), &fb2.message(),
                                                 &fbb));
      FlatbufferDetachedBuffer<Configuration> fb(fbb.Release());
      ExpectMergedOutput(fb, out);
    }
    ExpectMergedOutput(
        MergeFlatBuffers<Configuration>(&fb1.message(), &fb2.message()), out);
    ExpectMergedOutput(MergeFlatBuffers<Configuration>(fb1, fb2), out);

    // Now, to make things extra exciting, nest a config inside a config.  And
    // run all the tests.  This will exercise some fun nested merges and copies.

    {
      // in1_nested merged with "" => in1.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(fb1_nested, empty);
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in1_nested, FlatbufferToJson(fb_merged));
    }

    {
      // in2_nested merged with "" => in2_nested.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(fb2_nested, empty);
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in2_nested, FlatbufferToJson(fb_merged));
    }

    {
      // "" merged with in1_nested => in1_nested.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(empty, fb1_nested);
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in1_nested, FlatbufferToJson(fb_merged));
    }

    {
      // "" merged with in2_nested => in2_nested.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(empty, fb2_nested);
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in2_nested, FlatbufferToJson(fb_merged));
    }

    {
      // nullptr merged with in1_nested => in1_nested.
      FlatbufferDetachedBuffer<Configuration> fb_merged =
          MergeFlatBuffers<Configuration>(nullptr, &fb1_nested.message());
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in1_nested, FlatbufferToJson(fb_merged));
    }

    {
      // nullptr merged with in1_nested => in1_nested.
      FlatbufferDetachedBuffer<Configuration> fb_merged(
          MergeFlatBuffers<Configuration>(&fb1_nested.message(), nullptr));
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(in1_nested, FlatbufferToJson(fb_merged));
    }

    {
      // in1_nested merged with in2_nested => out_nested.
      FlatbufferDetachedBuffer<Configuration> fb_merged(
          MergeFlatBuffers<Configuration>(fb1_nested, fb2_nested));
      ASSERT_NE(fb_merged.span().size(), 0u);

      EXPECT_EQ(out_nested, FlatbufferToJson(fb_merged));
    }

    // TODO(austin): Try more flatbuffers...
    // Now try copying various flatbuffers and confirming they match.
    {
      flatbuffers::FlatBufferBuilder aos_flatbuffer_copy_fbb;
      aos_flatbuffer_copy_fbb.ForceDefaults(true);

      ABSL_LOG(INFO) << "Copying " << in1 << " "
                     << absl::BytesToHexString(FromFbb(fb1)) << " at "
                     << reinterpret_cast<const void *>(fb1.span().data())
                     << " size " << fb1.span().size();
      aos_flatbuffer_copy_fbb.Finish(CopyFlatBuffer<Configuration>(
          &fb1.message(), &aos_flatbuffer_copy_fbb));

      const aos::FlatbufferDetachedBuffer<Configuration> fb_copy(
          aos_flatbuffer_copy_fbb.Release());
      ASSERT_NE(fb_copy.span().size(), 0u);

      EXPECT_TRUE(fb1.Verify());

      ASSERT_TRUE(fb_copy.Verify()) << in1;

      EXPECT_EQ(in1, FlatbufferToJson(fb_copy));
    }

    {
      flatbuffers::FlatBufferBuilder aos_flatbuffer_copy_fbb;
      aos_flatbuffer_copy_fbb.ForceDefaults(false);

      ABSL_LOG(INFO) << "Copying without defaults " << in1 << " "
                     << absl::BytesToHexString(FromFbb(fb1)) << " at "
                     << reinterpret_cast<const void *>(fb1.span().data())
                     << " size " << fb1.span().size();
      aos_flatbuffer_copy_fbb.Finish(CopyFlatBuffer<Configuration>(
          &fb1.message(), &aos_flatbuffer_copy_fbb));

      const aos::FlatbufferDetachedBuffer<Configuration> fb_copy(
          aos_flatbuffer_copy_fbb.Release());
      ASSERT_NE(fb_copy.span().size(), 0u);

      EXPECT_TRUE(fb1.Verify());

      ASSERT_TRUE(fb_copy.Verify()) << in1;

      EXPECT_EQ(in1, FlatbufferToJson(fb_copy));
    }

    {
      flatbuffers::FlatBufferBuilder aos_flatbuffer_copy_message_ptr_fbb;
      aos_flatbuffer_copy_message_ptr_fbb.ForceDefaults(true);

      aos_flatbuffer_copy_message_ptr_fbb.Finish(CopyFlatBuffer<Configuration>(
          &fb2.message(), &aos_flatbuffer_copy_message_ptr_fbb));

      aos::FlatbufferDetachedBuffer<Configuration> fb_copy_message_ptr(
          aos_flatbuffer_copy_message_ptr_fbb.Release());
      ASSERT_NE(fb_copy_message_ptr.span().size(), 0u);

      flatbuffers::FlatBufferBuilder aos_flatbuffer_copy_full_fbb;
      aos_flatbuffer_copy_full_fbb.ForceDefaults(true);

      aos_flatbuffer_copy_full_fbb.Finish(BlindCopyFlatBuffer<Configuration>(
          aos::FlatbufferSpan<Configuration>(fb2.span()),
          &aos_flatbuffer_copy_full_fbb));

      aos::FlatbufferDetachedBuffer<Configuration> fb_copy_full(
          aos_flatbuffer_copy_full_fbb.Release());
      ASSERT_NE(fb_copy_full.span().size(), 0u);

      EXPECT_TRUE(fb2.Verify());

      ABSL_LOG(INFO) << "Verifying copy of " << in2;
      ASSERT_TRUE(fb_copy_message_ptr.Verify()) << in2;
      ABSL_LOG(INFO) << "Verifying full of " << in2;
      ASSERT_TRUE(fb_copy_full.Verify()) << in2;

      EXPECT_EQ(in2, FlatbufferToJson(fb_copy_message_ptr));
      EXPECT_EQ(in2, FlatbufferToJson(fb_copy_full));
    }

    {
      flatbuffers::FlatBufferBuilder aos_flatbuffer_copy_fbb;
      aos_flatbuffer_copy_fbb.ForceDefaults(true);

      aos_flatbuffer_copy_fbb.Finish(CopyFlatBuffer<Configuration>(
          &fb1_nested.message(), &aos_flatbuffer_copy_fbb));

      aos::FlatbufferDetachedBuffer<Configuration> fb_copy(
          aos_flatbuffer_copy_fbb.Release());
      ASSERT_NE(fb_copy.span().size(), 0u);

      ASSERT_TRUE(fb_copy.Verify());

      EXPECT_EQ(in1_nested, FlatbufferToJson(fb_copy));
    }

    {
      flatbuffers::FlatBufferBuilder aos_flatbuffer_copy_fbb;
      aos_flatbuffer_copy_fbb.ForceDefaults(true);

      aos_flatbuffer_copy_fbb.Finish(CopyFlatBuffer<Configuration>(
          &fb2_nested.message(), &aos_flatbuffer_copy_fbb));

      aos::FlatbufferDetachedBuffer<Configuration> fb_copy(
          aos_flatbuffer_copy_fbb.Release());
      ASSERT_NE(fb_copy.span().size(), 0u);

      ASSERT_TRUE(fb_copy.Verify());

      EXPECT_EQ(in2_nested, FlatbufferToJson(fb_copy));
    }
  }
};

// Test the easy ones.  Test every type, single, no nesting.
TEST_F(FlatbufferMerge, Basic) {
  JsonMerge("{  }", "{  }", "{  }");

  JsonMerge("{ \"foo_bool\": true }", "{ \"foo_bool\": false }",
            "{ \"foo_bool\": false }");

  JsonMerge("{ \"foo_bool\": false }", "{ \"foo_bool\": true }",
            "{ \"foo_bool\": true }");

  JsonMerge("{ \"foo_byte\": 5 }", "{ \"foo_byte\": -7 }",
            "{ \"foo_byte\": -7 }");

  JsonMerge("{ \"foo_ubyte\": 5 }", "{ \"foo_ubyte\": 7 }",
            "{ \"foo_ubyte\": 7 }");

  JsonMerge("{ \"foo_short\": 5 }", "{ \"foo_short\": 7 }",
            "{ \"foo_short\": 7 }");

  JsonMerge("{ \"foo_int\": 5 }", "{ \"foo_int\": 7 }", "{ \"foo_int\": 7 }");

  JsonMerge("{ \"foo_uint\": 5 }", "{ \"foo_uint\": 7 }",
            "{ \"foo_uint\": 7 }");

  JsonMerge("{ \"foo_long\": 5 }", "{ \"foo_long\": 7 }",
            "{ \"foo_long\": 7 }");

  JsonMerge("{ \"foo_ulong\": 5 }", "{ \"foo_ulong\": 7 }",
            "{ \"foo_ulong\": 7 }");

  JsonMerge("{ \"foo_float\": 5 }", "{ \"foo_float\": 7.1 }",
            "{ \"foo_float\": 7.1 }");

  JsonMerge("{ \"foo_double\": 5 }", "{ \"foo_double\": 7.1 }",
            "{ \"foo_double\": 7.1 }");
}

// Test arrays of simple types.
TEST_F(FlatbufferMerge, Array) {
  JsonMerge("{ \"foo_string\": \"baz\" }", "{ \"foo_string\": \"bar\" }",
            "{ \"foo_string\": \"bar\" }");

  JsonMerge("{ \"vector_foo_bool\": [ true, true, false ] }",
            "{ \"vector_foo_bool\": [ false, true, true, false ] }",
            "{ \"vector_foo_bool\": [ true, true, false, false, "
            "true, true, false ] }");

  JsonMerge("{ \"vector_foo_byte\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_byte\": [ -3, 1, 3, 2 ] }",
            "{ \"vector_foo_byte\": [ 9, 7, 1, -3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_ubyte\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_ubyte\": [ 3, 1, 3, 2 ] }",
            "{ \"vector_foo_ubyte\": [ 9, 7, 1, 3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_short\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_short\": [ -3, 1, 3, 2 ] }",
            "{ \"vector_foo_short\": [ 9, 7, 1, -3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_ushort\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_ushort\": [ 3, 1, 3, 2 ] }",
            "{ \"vector_foo_ushort\": [ 9, 7, 1, 3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_int\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_int\": [ -3, 1, 3, 2 ] }",
            "{ \"vector_foo_int\": [ 9, 7, 1, -3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_uint\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_uint\": [ 3, 1, 3, 2 ] }",
            "{ \"vector_foo_uint\": [ 9, 7, 1, 3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_long\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_long\": [ -3, 1, 3, 2 ] }",
            "{ \"vector_foo_long\": [ 9, 7, 1, -3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_ulong\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_ulong\": [ 3, 1, 3, 2 ] }",
            "{ \"vector_foo_ulong\": [ 9, 7, 1, 3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_float\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_float\": [ -3, 1.3, 3, 2 ] }",
            "{ \"vector_foo_float\": [ 9, 7, 1, -3, 1.3, 3, 2 ] }");

  JsonMerge(
      "{ \"vector_foo_string\": [ \"9\", \"7\", \"1 \" ] }",
      "{ \"vector_foo_string\": [ \"31\", \"32\" ] }",
      "{ \"vector_foo_string\": [ \"9\", \"7\", \"1 \", \"31\", \"32\" ] }");
}

// Test nested messages, and arrays of nested messages.
TEST_F(FlatbufferMerge, NestedStruct) {
  JsonMerge(
      "{ \"single_application\": { \"name\": \"woot\" } }",
      "{ \"single_application\": { \"name\": \"wow\", \"priority\": 7 } }",
      "{ \"single_application\": { \"name\": \"wow\", \"priority\": 7 } }");

  JsonMerge(
      "{ \"single_application\": { \"name\": \"wow\", \"priority\": 7 } }",
      "{ \"single_application\": { \"name\": \"woot\" } }",
      "{ \"single_application\": { \"name\": \"woot\", \"priority\": 7 } }");

  JsonMerge("{ \"apps\": [ { \"name\": \"woot\" }, { \"name\": \"wow\" } ] }",
            "{ \"apps\": [ { \"name\": \"woo2\" }, { \"name\": \"wo3\" } ] }",
            "{ \"apps\": [ { \"name\": \"woot\" }, { \"name\": \"wow\" }, { "
            "\"name\": \"woo2\" }, { \"name\": \"wo3\" } ] }");
}

// Test nested messages, and arrays of nested messages.
TEST(FlatbufferCopy, WholesaleCopy) {
  flatbuffers::FlatBufferBuilder fbb_expected;
  fbb_expected.ForceDefaults(true);
  fbb_expected.DedupVtables(false);

  {
    flatbuffers::Offset<flatbuffers::String> name1_offset =
        fbb_expected.CreateString("wow");

    std::vector<flatbuffers::Offset<Application>> application_offsets;
    Application::Builder a1_builder(fbb_expected);
    a1_builder.add_name(name1_offset);
    a1_builder.add_priority(7);
    a1_builder.add_long_thingy(0x2546493132LL);
    application_offsets.emplace_back(a1_builder.Finish());

    flatbuffers::Offset<flatbuffers::String> name2_offset =
        fbb_expected.CreateString("foo");
    Application::Builder a2_builder(fbb_expected);
    a2_builder.add_name(name2_offset);
    a2_builder.add_priority(3);
    a2_builder.add_long_thingy(9);
    application_offsets.emplace_back(a2_builder.Finish());

    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Application>>>
        applications_offset = fbb_expected.CreateVector(application_offsets);

    Configuration::Builder configuration_builder(fbb_expected);
    configuration_builder.add_apps(applications_offset);
    fbb_expected.Finish(configuration_builder.Finish());
  }

  ABSL_LOG(INFO) << "Initial alignment "
                 << fbb_expected.GetBufferMinAlignment();

  aos::FlatbufferDetachedBuffer<Configuration> expected(fbb_expected.Release());

  ABSL_LOG(INFO) << "Expected "
                 << absl::BytesToHexString(std::string_view(
                        reinterpret_cast<char *>(expected.span().data()),
                        expected.span().size()));

  aos::FlatbufferDetachedBuffer<Application> a1 = []() {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);

    flatbuffers::Offset<flatbuffers::String> name1_offset =
        fbb.CreateString("wow");

    Application::Builder a1_builder(fbb);
    a1_builder.add_name(name1_offset);
    a1_builder.add_priority(7);
    a1_builder.add_long_thingy(0x2546493132LL);
    flatbuffers::Offset<Application> a1 = a1_builder.Finish();

    fbb.Finish(a1);
    return fbb.Release();
  }();

  aos::FlatbufferDetachedBuffer<Application> a2 =
      JsonToFlatbuffer<Application>(static_cast<const char *>(
          "{ \"name\": \"foo\", \"priority\": 3, \"long_thingy\": 9 }"));

  aos::FlatbufferDetachedBuffer<Configuration> c1 = [&a1, &a2]() {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);

    std::vector<flatbuffers::Offset<Application>> application_offsets;
    application_offsets.emplace_back(BlindCopyFlatBuffer(a1, &fbb));
    application_offsets.emplace_back(BlindCopyFlatBuffer(a2, &fbb));

    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Application>>>
        applications_offset = fbb.CreateVector(application_offsets);

    Configuration::Builder configuration_builder(fbb);
    configuration_builder.add_apps(applications_offset);
    fbb.Finish(configuration_builder.Finish());

    return fbb.Release();
  }();

  ABSL_LOG(INFO) << "Got      "
                 << absl::BytesToHexString(std::string_view(
                        reinterpret_cast<char *>(c1.span().data()),
                        c1.span().size()));

  aos::FlatbufferDetachedBuffer<Configuration> c2 = [&a1, &a2]() {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);

    std::vector<flatbuffers::Offset<Application>> application_offsets;
    application_offsets.emplace_back(CopyFlatBuffer(&a1.message(), &fbb));
    application_offsets.emplace_back(CopyFlatBuffer(&a2.message(), &fbb));

    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Application>>>
        applications_offset = fbb.CreateVector(application_offsets);

    Configuration::Builder configuration_builder(fbb);
    configuration_builder.add_apps(applications_offset);
    fbb.Finish(configuration_builder.Finish());

    return fbb.Release();
  }();

  ABSL_LOG(INFO) << "Got      "
                 << absl::BytesToHexString(std::string_view(
                        reinterpret_cast<char *>(c2.span().data()),
                        c2.span().size()));

  ASSERT_TRUE(expected.Verify());
  ASSERT_TRUE(c1.Verify());
  ASSERT_TRUE(c2.Verify());

  ABSL_LOG(INFO) << FlatbufferToJson(expected);
  ABSL_LOG(INFO) << FlatbufferToJson(c1);
  ABSL_LOG(INFO) << FlatbufferToJson(c2);
  EXPECT_EQ(FlatbufferToJson(expected), FlatbufferToJson(c1));
  EXPECT_EQ(FlatbufferToJson(expected), FlatbufferToJson(c2));
}

// Tests for the realtime-safe, reflection-based CompareFlatBuffer
// implementation.  Each comparison runs inside aos::ScopedRealtime so that any
// hidden allocation inside CompareFlatBuffer trips the malloc hook and aborts
// the test.
class FlatbufferCompare : public ::testing::Test {
 protected:
  static aos::FlatbufferDetachedBuffer<Configuration> ParseTestFb(
      const char *json) {
    return aos::JsonToFlatbuffer(json, ConfigurationTypeTable());
  }

  // Runs CompareFlatBuffer under aos::ScopedRealtime so that any malloc inside
  // will trip the hook and crash this test.  Disabled under asan/msan since
  // the malloc hook is incompatible with those sanitizers.
  static bool CompareInRealtime(const Configuration *a,
                                const Configuration *b) {
#if !defined(AOS_SANITIZE_MEMORY) && !defined(AOS_SANITIZE_ADDRESS)
    aos::ScopedRealtime realtime;
#endif
    return CompareFlatBuffer(a, b);
  }
};

using FlatbufferCompareDeathTest = FlatbufferCompare;

// TODO(austin): unions
// TODO(austin): struct

// --- Depth 0: scalars, enums, strings, and field presence ------------------

TEST_F(FlatbufferCompare, CompareEqualScalarsAndEnums) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "foo_int": 7, "foo_string": "hi",
                       "foo_enum": "UType", "foo_double": 1.5 })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "foo_int": 7, "foo_string": "hi",
                       "foo_enum": "UType", "foo_double": 1.5 })");
  EXPECT_TRUE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareEqualScalarsAndEnumsInDifferentOrder) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "foo_double": 1.5, "foo_enum": "UType",
                       "foo_int": 7, "foo_string": "hi" })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "foo_int": 7, "foo_string": "hi",
                       "foo_enum": "UType", "foo_double": 1.5 })");
  EXPECT_TRUE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentScalar) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "foo_int": 7 })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "foo_int": 8 })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentEnum) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "foo_enum": "UType" })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "foo_enum": "Bool" })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentEnumNonConsecutive) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "foo_enum_nonconsecutive": "Zero" })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "foo_enum_nonconsecutive": "Big" })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentString) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "foo_string": "abc" })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "foo_string": "abd" })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentStringLength) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "foo_string": "abc" })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "foo_string": "abcd" })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

// Presence semantic: an absent field is distinct from one explicitly set to
// the default value.
TEST_F(FlatbufferCompare, CompareEnumAbsentVsDefault) {
  aos::FlatbufferDetachedBuffer<Configuration> absent = ParseTestFb(R"({})");
  aos::FlatbufferDetachedBuffer<Configuration> present_default =
      ParseTestFb(R"({ "foo_enum_default": "None" })");
  EXPECT_FALSE(
      CompareInRealtime(&absent.message(), &present_default.message()));
}

// --- Depth 1: vectors of scalars, enums, and strings -----------------------

TEST_F(FlatbufferCompare, CompareDifferentScalarVectorContent) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "vector_foo_int": [1, 2, 3] })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "vector_foo_int": [1, 2, 4] })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentScalarVectorLength) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "vector_foo_int": [1, 2, 3] })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "vector_foo_int": [1, 2, 3, 4] })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareEqualEmptyVector) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "vector_foo_int": [] })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "vector_foo_int": [] })");
  EXPECT_TRUE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentEnumVector) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "vector_foo_enum": ["UType", "Bool"] })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "vector_foo_enum": ["UType", "Byte"] })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentStringVector) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "vector_foo_string": ["x", "y"] })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "vector_foo_string": ["x", "z"] })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentStringVectorLength) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "vector_foo_string": ["x"] })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "vector_foo_string": ["x", "y"] })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

// --- Depth 2: sub-tables and vectors of sub-tables -------------------------

TEST_F(FlatbufferCompare, CompareDifferentSubTableScalar) {
  aos::FlatbufferDetachedBuffer<Configuration> a = ParseTestFb(
      R"({ "single_application": { "name": "x", "priority": 1 } })");
  aos::FlatbufferDetachedBuffer<Configuration> b = ParseTestFb(
      R"({ "single_application": { "name": "x", "priority": 2 } })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentSubTableString) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "single_application": { "name": "alpha" } })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "single_application": { "name": "beta" } })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentVectorOfTablesLength) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "apps": [ { "name": "a", "priority": 1 } ] })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "apps": [ { "name": "a", "priority": 1 },
                              { "name": "b", "priority": 2 } ] })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentVectorOfTablesElement) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "apps": [ { "name": "a", "priority": 1 },
                                 { "name": "b", "priority": 2 } ] })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "apps": [ { "name": "a", "priority": 1 },
                                 { "name": "b", "priority": 3 } ] })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

// --- Depth 3+: deeply nested and vector-of-vector-of-string ----------------

TEST_F(FlatbufferCompare, CompareDifferentDeeplyNested) {
  aos::FlatbufferDetachedBuffer<Configuration> a = ParseTestFb(R"({
    "nested_config": {
      "single_application": { "name": "alpha", "priority": 1 }
    }
  })");
  aos::FlatbufferDetachedBuffer<Configuration> b = ParseTestFb(R"({
    "nested_config": {
      "single_application": { "name": "beta", "priority": 1 }
    }
  })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareEqualDeeplyNested) {
  const char *json = R"({
    "nested_config": {
      "single_application": { "name": "alpha", "priority": 1 },
      "apps": [ { "name": "a", "priority": 1 } ]
    }
  })";
  aos::FlatbufferDetachedBuffer<Configuration> a = ParseTestFb(json);
  aos::FlatbufferDetachedBuffer<Configuration> b = ParseTestFb(json);
  EXPECT_TRUE(CompareInRealtime(&a.message(), &b.message()));
}

TEST_F(FlatbufferCompare, CompareDifferentVectorOfVectorOfStrings) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "vov": { "v": [ { "str": ["a", "b"] },
                                       { "str": ["c", "d"] } ] } })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "vov": { "v": [ { "str": ["a", "b"] },
                                       { "str": ["c", "e"] } ] } })");
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

// Validate null-safety.
TEST_F(FlatbufferCompare, CompareNullPointers) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "foo_int": 1 })");
  EXPECT_TRUE(CompareInRealtime(nullptr, nullptr));
  EXPECT_FALSE(CompareInRealtime(&a.message(), nullptr));
  EXPECT_FALSE(CompareInRealtime(nullptr, &a.message()));
}

// --- NaN handling ----------------------------------------------------------
//
// CompareFlatBuffer uses memcmp on scalar bytes, so two identical NaN bit
// patterns must compare equal (unlike IEEE 754 `==`).  We build the
// flatbuffers programmatically because JSON has no NaN literal.

namespace {

// Helper to build a Configuration with the given float/double values set.
aos::FlatbufferDetachedBuffer<Configuration> BuildFloatConfig(
    float foo_float, double foo_double) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  ConfigurationBuilder builder(fbb);
  builder.add_foo_float(foo_float);
  builder.add_foo_double(foo_double);
  fbb.Finish(builder.Finish());
  return aos::FlatbufferDetachedBuffer<Configuration>(fbb.Release());
}

}  // namespace

// Both sides use quiet_NaN -- identical bit patterns must compare equal.
TEST_F(FlatbufferCompare, CompareNaNQuietEqual) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      BuildFloatConfig(std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
  aos::FlatbufferDetachedBuffer<Configuration> b =
      BuildFloatConfig(std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(CompareInRealtime(&a.message(), &b.message()));
}

// std::nanf("") / std::nan("") should be the same as quiet NaNs.
TEST_F(FlatbufferCompare, CompareNaNStdNanEqual) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      BuildFloatConfig(std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
  aos::FlatbufferDetachedBuffer<Configuration> b =
      BuildFloatConfig(std::nanf(""), std::nan(""));
  EXPECT_TRUE(CompareInRealtime(&a.message(), &b.message()));
}

// NAN should be the same as quiet NaNs.
TEST_F(FlatbufferCompare, CompareNaNMacroEqual) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      BuildFloatConfig(std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
  aos::FlatbufferDetachedBuffer<Configuration> b = BuildFloatConfig(NAN, NAN);
  EXPECT_TRUE(CompareInRealtime(&a.message(), &b.message()));
}

// Both sides use signaling_NaN -- also identical bit patterns.
TEST_F(FlatbufferCompare, CompareNaNSignalingEqual) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      BuildFloatConfig(std::numeric_limits<float>::signaling_NaN(),
                       std::numeric_limits<double>::signaling_NaN());
  aos::FlatbufferDetachedBuffer<Configuration> b =
      BuildFloatConfig(std::numeric_limits<float>::signaling_NaN(),
                       std::numeric_limits<double>::signaling_NaN());
  EXPECT_TRUE(CompareInRealtime(&a.message(), &b.message()));
}

// quiet_NaN vs signaling_NaN have different bit patterns -- must compare
// unequal.
TEST_F(FlatbufferCompare, CompareNaNQuietVsSignalingNotEqual) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      BuildFloatConfig(std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
  aos::FlatbufferDetachedBuffer<Configuration> b =
      BuildFloatConfig(std::numeric_limits<float>::signaling_NaN(),
                       std::numeric_limits<double>::signaling_NaN());
  EXPECT_FALSE(CompareInRealtime(&a.message(), &b.message()));
}

// --- Death tests -----------------------------------------------------------

// Struct fields are explicitly unsupported and must abort.
TEST_F(FlatbufferCompareDeathTest, StructFieldAborts) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "foo_struct": { "foo_byte": 1,
                                       "nested_struct": { "foo_byte": 2 } } })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "foo_struct": { "foo_byte": 1,
                                       "nested_struct": { "foo_byte": 2 } } })");

  EXPECT_DEATH(
      { CompareFlatBuffer(&a.message(), &b.message()); },
      "CompareFlatBuffer only supports sub-tables");
}

// Vectors of structs are likewise unsupported.
TEST_F(FlatbufferCompareDeathTest, VectorOfStructFieldAborts) {
  aos::FlatbufferDetachedBuffer<Configuration> a =
      ParseTestFb(R"({ "vector_foo_struct": [
                      { "foo_byte": 1, "nested_struct": { "foo_byte": 2 } }
                    ] })");
  aos::FlatbufferDetachedBuffer<Configuration> b =
      ParseTestFb(R"({ "vector_foo_struct": [
                      { "foo_byte": 1, "nested_struct": { "foo_byte": 2 } }
                    ] })");

  EXPECT_DEATH(
      { CompareFlatBuffer(&a.message(), &b.message()); },
      "CompareFlatBuffer only supports vectors of tables");
}

}  // namespace aos::testing
