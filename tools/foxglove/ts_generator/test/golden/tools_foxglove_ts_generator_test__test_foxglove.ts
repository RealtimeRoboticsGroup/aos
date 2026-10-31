// Auto-generated Foxglove-compatible TypeScript types.
// Source: test.fbs
// DO NOT EDIT.

import { Metadata } from './tools_foxglove_ts_generator_test__imported_foxglove';

export enum Color {
	Red = 0,
	Green = 1,
	Blue = 2,
}

export enum Priority {
	Low = 0,
	Medium = 1,
	High = 2,
}

export type Position = {
	x?: number;
	y?: number;
	z?: number;
};

export type TestMessage = {
	name?: string;
	value?: number;
	active?: boolean;
	score?: number;
	timestamp?: bigint;
	color?: Color;
	data?: Uint8Array;
	tags?: string[];
	position?: Position;
	scores?: number[];
	// Serialized as Uint8Array because the Color enum's underlying type is `ubyte`.
	colors?: Uint8Array;
	priority?: Priority;
	priorities?: Priority[];
	metadata?: Metadata;
};

