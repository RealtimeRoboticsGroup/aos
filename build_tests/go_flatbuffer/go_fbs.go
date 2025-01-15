package main

import (
	build_tests "github.com/RealtimeRoboticsGroup/aos/build_tests/fbs"
	flatbuffers "github.com/google/flatbuffers/go"
)

func main() {
	builder := flatbuffers.NewBuilder(1024)
	build_tests.FooStart(builder)
	build_tests.FooAddValue(builder, 3)
	build_tests.FooEnd(builder)
}
