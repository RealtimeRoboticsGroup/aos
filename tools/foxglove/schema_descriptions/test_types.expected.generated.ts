// AUTO-GENERATED FILE — do not edit by hand.
// Generated from test_types.ts by tools/foxglove/schema_descriptions.
// Regenerate by rebuilding the surrounding Bazel target.

import { MessageSchemaDescription } from '@foxglove/extension';

export const SimplePrimitivesSchemaDescription: MessageSchemaDescription = {
	name: "string",
	count: "number",
	active: "bool",
	raw_data: ["byte"],
};

export const ArraysAndOptionalSchemaDescription: MessageSchemaDescription = {
	labels: ["string"],
	weights: ["number"],
	flags: ["bool"],
};

export const NestedSchemaDescription: MessageSchemaDescription = {
	position: {
		x: "number",
		y: "number",
	},
	waypoints: [{
		id: "string",
		time: "number",
	}],
};
