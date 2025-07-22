from absl.testing import absltest
import flatbuffers


class TestPythonFlatbuffers(absltest.TestCase):

    def test_local_flatbuffer_all_fields_set(self):
        """Serialize and deserialize a local flatbuffer, setting all its fields.

        This flatbuffer includes a local flatbuffer, a generated flatbuffer, and an
        external flatbuffer (transitively).
        """
        import everything_fbs_py.aos.flatbuffers.MainTable as MainTableFbs
        import everything_fbs_py.aos.flatbuffers.fbs.MyEnum as MyEnumFbs
        import everything_fbs_py.aos.flatbuffers.fbs.SubStruct as SubStructFbs
        import everything_fbs_py.aos.flatbuffers.fbs.SubTable as SubTableFbs
        import everything_fbs_py.aos.flatbuffers.fbs.GeneratedTable as GeneratedTableFbs
        import everything_fbs_py.aos.flatbuffers.test_dir.IncludedTable as IncludedTableFbs
        import everything_fbs_py.tests.fbs.Foo as FooFbs
        import everything_fbs_py.include.fbs.Bar as BarFbs

        fbb = flatbuffers.Builder()

        MainTableFbs.StartVectorOfScalarsVector(fbb, 10)
        for i in reversed(range(0, 10)):
            fbb.PrependInt32(i)
        vector_of_scalars = fbb.EndVector()

        string = fbb.CreateString("lonely string".encode("utf-8"))

        strings = [
            fbb.CreateString(f"my_string_{i}".encode("utf-8"))
            for i in range(1, 11)
        ]
        MainTableFbs.StartVectorOfStringsVector(fbb, 10)
        for s in reversed(strings):
            fbb.PrependUOffsetTRelative(s)
        vector_of_strings = fbb.EndVector()

        SubTableFbs.Start(fbb)
        SubTableFbs.AddFoo(fbb, 10)
        with self.assertRaises(AttributeError):
            # This field is deprecated in the schema.
            SubTableFbs.AddBar(fbb, 15)
        SubTableFbs.AddBaz(fbb, 20)
        sub_table = SubTableFbs.End(fbb)

        MainTableFbs.StartVectorOfStructsVector(fbb, 3)
        SubStructFbs.CreateSubStruct(fbb, 9.10, 11.12)
        SubStructFbs.CreateSubStruct(fbb, 5.6, 7.8)
        SubStructFbs.CreateSubStruct(fbb, 1.2, 3.4)
        vector_of_structs = fbb.EndVector()

        tables = []
        MAX_32_BIT_SIGNED = 2_147_483_647
        for i in reversed(range(1, 6)):
            SubTableFbs.Start(fbb)
            SubTableFbs.AddFoo(fbb, 10 * i)
            SubTableFbs.AddBaz(fbb, MAX_32_BIT_SIGNED + i)
            tables.append(SubTableFbs.End(fbb))
        MainTableFbs.StartVectorOfTablesVector(fbb, 5)
        for table in tables:
            fbb.PrependUOffsetTRelative(table)
        vector_of_tables = fbb.EndVector()

        BarFbs.Start(fbb)
        BarFbs.AddValue(fbb, 42)
        bar = BarFbs.End(fbb)

        FooFbs.Start(fbb)
        FooFbs.AddBar(fbb, bar)
        foo = FooFbs.End(fbb)

        IncludedTableFbs.Start(fbb)
        IncludedTableFbs.AddFoo(fbb, foo)
        included_table = IncludedTableFbs.End(fbb)

        GeneratedTableFbs.Start(fbb)
        GeneratedTableFbs.AddValue(fbb, 27)
        generated_table = GeneratedTableFbs.End(fbb)

        MainTableFbs.Start(fbb)
        MainTableFbs.AddVectorOfScalars(fbb, vector_of_scalars)
        MainTableFbs.AddString(fbb, string)
        MainTableFbs.AddVectorOfStrings(fbb, vector_of_strings)
        MainTableFbs.AddEnum(fbb, MyEnumFbs.MyEnum.BAR)
        MainTableFbs.AddSubStruct(
            fbb, SubStructFbs.CreateSubStruct(fbb, 13.14, 15.16))
        MainTableFbs.AddSubTable(fbb, sub_table)
        MainTableFbs.AddVectorOfStructs(fbb, vector_of_structs)
        MainTableFbs.AddVectorOfTables(fbb, vector_of_tables)
        MainTableFbs.AddIncludedTable(fbb, included_table)
        MainTableFbs.AddGeneratedTable(fbb, generated_table)
        main_table = MainTableFbs.End(fbb)
        self.assertNotEqual(main_table, 0)

        fbb.Finish(main_table)
        buf = fbb.Output()

        # Deserialize it and check its fields.
        main_table = MainTableFbs.MainTable.GetRootAs(buf, 0)

        self.assertEqual(main_table.Scalar(), 99)

        self.assertFalse(main_table.VectorOfScalarsIsNone())
        self.assertEqual(main_table.VectorOfScalarsLength(), 10)
        for i in range(0, 10):
            self.assertEqual(main_table.VectorOfScalars(i), i)

        self.assertIsNotNone(main_table.String())
        self.assertEqual(main_table.String(), "lonely string".encode("utf-8"))

        self.assertFalse(main_table.VectorOfStringsIsNone())
        self.assertEqual(main_table.VectorOfStringsLength(), 10)
        for i in range(0, 10):
            self.assertEqual(main_table.VectorOfStrings(i),
                             f"my_string_{i + 1}".encode("utf-8"))

        self.assertEqual(main_table.Enum(), MyEnumFbs.MyEnum.BAR)

        self.assertIsNotNone(main_table.SubStruct())
        self.assertEqual(main_table.SubStruct().X(), 13.14)
        self.assertEqual(main_table.SubStruct().Y(), 15.16)

        self.assertIsNotNone(main_table.SubTable())
        self.assertEqual(main_table.SubTable().Foo(), 10)
        self.assertEqual(main_table.SubTable().Baz(), 20)

        self.assertFalse(main_table.VectorOfStructsIsNone())
        self.assertEqual(main_table.VectorOfStructsLength(), 3)
        self.assertEqual(main_table.VectorOfStructs(0).X(), 1.2)
        self.assertEqual(main_table.VectorOfStructs(0).Y(), 3.4)
        self.assertEqual(main_table.VectorOfStructs(1).X(), 5.6)
        self.assertEqual(main_table.VectorOfStructs(1).Y(), 7.8)
        self.assertEqual(main_table.VectorOfStructs(2).X(), 9.10)
        self.assertEqual(main_table.VectorOfStructs(2).Y(), 11.12)

        self.assertFalse(main_table.VectorOfTablesIsNone())
        self.assertEqual(main_table.VectorOfTablesLength(), 5)
        for i in range(0, 5):
            table = main_table.VectorOfTables(i)
            self.assertEqual(table.Foo(), 10 * (i + 1))
            self.assertEqual(table.Baz(), MAX_32_BIT_SIGNED + i + 1)
            with self.assertRaises(AttributeError):
                # TODO(Sanjay): Why are accessors not available for deprecated fields?
                table.Bar()

        self.assertIsNotNone(main_table.IncludedTable())
        self.assertIsNotNone(main_table.IncludedTable().Foo())
        self.assertIsNotNone(main_table.IncludedTable().Foo().Bar())
        self.assertEqual(main_table.IncludedTable().Foo().Bar().Value(), 42)

        self.assertIsNotNone(main_table.GeneratedTable())
        self.assertEqual(main_table.GeneratedTable().Value(), 27)

    def test_local_flatbuffers_no_fields_set(self):
        """Serialize and deserialize a local flatbuffer, leaving all the fields unset."""
        import everything_fbs_py.aos.flatbuffers.MainTable as MainTableFbs

        # Serialize an empty MainTable.
        fbb = flatbuffers.Builder()
        MainTableFbs.Start(fbb)
        fbb.Finish(MainTableFbs.End(fbb))
        buf = fbb.Output()

        # Deserialize it and verify its contents.
        main_table = MainTableFbs.MainTable.GetRootAs(buf, 0)
        self.assertEqual(main_table.Scalar(), 99)
        self.assertTrue(main_table.VectorOfScalarsIsNone())
        self.assertIsNone(main_table.String())
        self.assertTrue(main_table.VectorOfStringsIsNone())
        self.assertEqual(main_table.Enum(), 0)
        self.assertIsNone(main_table.SubStruct())
        self.assertIsNone(main_table.SubTable())
        self.assertTrue(main_table.VectorOfStructsIsNone())
        self.assertTrue(main_table.VectorOfTablesIsNone())
        self.assertIsNone(main_table.IncludedTable())
        self.assertIsNone(main_table.GeneratedTable())

    def test_generated_flatbuffer(self):
        """Serialize and deserialize a generated flatbuffer."""
        import generated_fbs_py.aos.flatbuffers.fbs.GeneratedTable as GeneratedTableFbs

        # Serialize a GeneratedTable.
        fbb = flatbuffers.Builder()
        GeneratedTableFbs.Start(fbb)
        GeneratedTableFbs.AddValue(fbb, 42)
        fbb.Finish(GeneratedTableFbs.End(fbb))
        buf = fbb.Output()

        # Deserialize it.
        generated_table = GeneratedTableFbs.GeneratedTable.GetRootAs(buf, 0)
        self.assertEqual(generated_table.Value(), 42)

    def test_external_flatbuffer_with_external_include(self):
        """Serialize and deserialize an external flatbuffer.

        This flatbuffer includes another flatbuffer in its own workspace.
        """
        import foo_fbs_py.tests.fbs.Foo as FooFbs
        import foo_fbs_py.include.fbs.Bar as BarFbs

        # Serialize a Foo.
        fbb = flatbuffers.Builder()

        BarFbs.Start(fbb)
        BarFbs.AddValue(fbb, 42)
        bar = BarFbs.End(fbb)
        self.assertNotEqual(bar, 0)

        FooFbs.Start(fbb)
        FooFbs.AddBar(fbb, bar)
        foo = FooFbs.End(fbb)
        self.assertNotEqual(foo, 0)

        fbb.Finish(foo)
        buf = fbb.Output()

        # Deserialize it.
        foo = FooFbs.Foo.GetRootAs(buf, 0)
        self.assertIsNotNone(foo.Bar())
        self.assertEqual(foo.Bar().Value(), 42)


if __name__ == "__main__":
    absltest.main()
