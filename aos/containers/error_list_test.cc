#include "aos/containers/error_list.h"

#include <stdint.h>

#include <memory>

#include "gtest/gtest.h"

#include "aos/json_to_flatbuffer_generated.h"

namespace aos::testing {

enum class TestEnum : int8_t {
  FOO = 0,
  BAR = 1,
  BAZ = 2,
  VWEEP = 3,
  MIN = FOO,
  MAX = VWEEP
};

// Tests that setting works and allows no duplicates
TEST(ErrorListTest, NoDuplicates) {
  ErrorList<TestEnum> a;
  EXPECT_EQ(a.size(), 0);
  a.Set(TestEnum::BAZ);
  EXPECT_EQ(a.at(0), TestEnum::BAZ);
  EXPECT_EQ(a.size(), 1);
  a.Set(TestEnum::BAZ);
  EXPECT_EQ(a.at(0), TestEnum::BAZ);
  EXPECT_EQ(a.size(), 1);
  a.Set(TestEnum::VWEEP);
  EXPECT_EQ(a.at(0), TestEnum::BAZ);
  EXPECT_EQ(a.at(1), TestEnum::VWEEP);
  EXPECT_EQ(a.size(), 2);
  a.Set(TestEnum::FOO);
  EXPECT_EQ(a.at(0), TestEnum::FOO);
  EXPECT_EQ(a.at(1), TestEnum::BAZ);
  EXPECT_EQ(a.at(2), TestEnum::VWEEP);
  EXPECT_EQ(a.size(), 3);
}

// Tests that clearing works
TEST(ErrorListTest, Clearing) {
  ErrorList<TestEnum> a;
  a.Set(TestEnum::FOO);
  a.Set(TestEnum::BAZ);
  a.Set(TestEnum::VWEEP);
  EXPECT_EQ(a.at(0), TestEnum::FOO);
  EXPECT_EQ(a.at(1), TestEnum::BAZ);
  EXPECT_EQ(a.at(2), TestEnum::VWEEP);
  EXPECT_EQ(a.size(), 3);

  a.Clear(TestEnum::BAR);
  EXPECT_EQ(a.at(0), TestEnum::FOO);
  EXPECT_EQ(a.at(1), TestEnum::BAZ);
  EXPECT_EQ(a.at(2), TestEnum::VWEEP);
  EXPECT_EQ(a.size(), 3);

  a.Clear(TestEnum::BAZ);
  EXPECT_EQ(a.at(0), TestEnum::FOO);
  EXPECT_EQ(a.at(1), TestEnum::VWEEP);
  EXPECT_EQ(a.size(), 2);
}

// Tests that clearing all errors works.
TEST(ErrorListTest, ClearAll) {
  // Set up a list to test with.
  ErrorList<TestEnum> a;
  a.Set(TestEnum::FOO);
  a.Set(TestEnum::BAZ);
  a.Set(TestEnum::VWEEP);
  EXPECT_EQ(a.size(), 3);
  EXPECT_FALSE(a.empty());
  EXPECT_TRUE(a.Has(TestEnum::FOO));
  EXPECT_TRUE(a.Has(TestEnum::BAZ));
  EXPECT_TRUE(a.Has(TestEnum::VWEEP));

  // Validate that clearing everything does indeed leave the list empty.
  a.ClearAll();
  EXPECT_EQ(a.size(), 0);
  EXPECT_TRUE(a.empty());
  EXPECT_FALSE(a.Has(TestEnum::FOO));
  EXPECT_FALSE(a.Has(TestEnum::BAZ));
  EXPECT_FALSE(a.Has(TestEnum::VWEEP));

  // Clearing an empty list should be idempotent.
  a.ClearAll();
  EXPECT_EQ(a.size(), 0);
  EXPECT_TRUE(a.empty());

  // A cleared list should still be usable.
  a.Set(TestEnum::BAR);
  a.Set(TestEnum::FOO);
  EXPECT_EQ(a.size(), 2);
  EXPECT_EQ(a.at(0), TestEnum::FOO);
  EXPECT_EQ(a.at(1), TestEnum::BAR);
  EXPECT_TRUE(a.Has(TestEnum::BAR));
  EXPECT_TRUE(a.Has(TestEnum::FOO));
}

// Tests that checking for a value works
TEST(ErrorListTest, Has) {
  ErrorList<TestEnum> a;
  a.Set(TestEnum::FOO);
  a.Set(TestEnum::BAZ);
  a.Set(TestEnum::VWEEP);
  EXPECT_EQ(a.at(0), TestEnum::FOO);
  EXPECT_EQ(a.at(1), TestEnum::BAZ);
  EXPECT_EQ(a.at(2), TestEnum::VWEEP);
  EXPECT_EQ(a.size(), 3);

  EXPECT_TRUE(a.Has(TestEnum::FOO));
  EXPECT_TRUE(a.Has(TestEnum::VWEEP));
  EXPECT_TRUE(a.Has(TestEnum::BAZ));
  EXPECT_FALSE(a.Has(TestEnum::BAR));
}

// Tests serializing and deserializing to/from flatbuffers.
TEST(ErrorListTest, Flatbuffers) {
  ErrorList<BaseType> a;
  a.Set(BaseType::Bool);
  a.Set(BaseType::Float);
  a.Set(BaseType::Short);
  EXPECT_TRUE(a.Has(BaseType::Bool));
  EXPECT_TRUE(a.Has(BaseType::Short));
  EXPECT_TRUE(a.Has(BaseType::Float));
  EXPECT_EQ(a.at(0), BaseType::Bool);
  EXPECT_EQ(a.at(1), BaseType::Short);
  EXPECT_EQ(a.at(2), BaseType::Float);
  EXPECT_EQ(a.size(), 3);

  flatbuffers::FlatBufferBuilder fbb(1024);
  flatbuffers::Offset<flatbuffers::Vector<BaseType>> vector =
      a.ToFlatbuffer(&fbb);

  ConfigurationBuilder builder(fbb);
  builder.add_vector_foo_enum(vector);

  fbb.Finish(builder.Finish());
  const Configuration *config =
      flatbuffers::GetRoot<Configuration>(fbb.GetBufferPointer());

  ErrorList<BaseType> b(*config->vector_foo_enum());
  EXPECT_TRUE(b.Has(BaseType::Bool));
  EXPECT_TRUE(b.Has(BaseType::Short));
  EXPECT_TRUE(b.Has(BaseType::Float));
  EXPECT_EQ(b.at(0), BaseType::Bool);
  EXPECT_EQ(b.at(1), BaseType::Short);
  EXPECT_EQ(b.at(2), BaseType::Float);
  EXPECT_EQ(b.size(), 3);
}

}  // namespace aos::testing
