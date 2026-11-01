// The scene of reflex-trace/1 (docs/standards/animation, "Trace schema"), in
// the validator: the example trace's scene is valid, a trace without one is
// too, and each rule the schema or validate.js enforces rejects a scene that
// breaks it.
//
//   npm run test:validate

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { validateTrace } from '@branes-ai/trace-player/validate';

const example = JSON.parse(readFileSync(new URL('../../src/traces/player/scene_square.json', import.meta.url), 'utf8'));

/** The example with its scene changed by `edit`, and the validator's errors. */
function errorsAfter(edit) {
  const trace = structuredClone(example);
  edit(trace.scene.entities, trace);
  return validateTrace(trace);
}

const entity = (entities, id) => entities.find((e) => e.id === id);

test('the example scene is valid, and so is a trace without a scene', () => {
  assert.deepEqual(validateTrace(example), []);
  const roles = new Set(example.scene.entities.map((e) => e.role));
  for (const role of ['obstacle', 'goal', 'gate', 'boundary', 'marker']) assert.ok(roles.has(role), role);
  const bare = structuredClone(example);
  delete bare.scene;
  assert.deepEqual(validateTrace(bare), []);
});

test('the schema refuses a malformed entity', () => {
  const cases = {
    'an unknown role': (es) => (entity(es, 'tree-1').role = 'tree'),
    'an id outside the pattern': (es) => (entity(es, 'tree-1').id = 'Tree 1'),
    'a zero radius': (es) => (entity(es, 'tree-1').shape.radius = 0),
    'a box with two sides': (es) => (entity(es, 'rock').shape.size = [1, 1]),
    'an unknown shape': (es) => (entity(es, 'rock').shape = { type: 'cone', radius: 1 }),
    'a compound inside a compound': (es) => {
      const gate = entity(es, 'gate');
      gate.shape.parts[0].shape = structuredClone(gate.shape);
    },
    'a pose without a position': (es) => delete entity(es, 'goal').pose.position,
    'an unknown visual': (es) => (entity(es, 'tree-1').visual = { kind: 'bush' }),
    'a model without a uri': (es) => (entity(es, 'rock').visual = { kind: 'model' }),
    'an extra key': (es) => (entity(es, 'goal').colour = 'green'),
    'an empty scene': (es, trace) => (trace.scene.entities = []),
  };
  for (const [name, edit] of Object.entries(cases)) {
    const errors = errorsAfter(edit);
    assert.ok(errors.length > 0 && errors.every((e) => e.startsWith('schema:')), `${name}: ${errors.join('; ')}`);
  }
});

test('the validator refuses what the schema cannot express', () => {
  const cases = {
    'a duplicate id': [(es) => (entity(es, 'tree-2').id = 'tree-1'), /scene entity tree-1: duplicate id/],
    'a tree on a box': [(es) => (entity(es, 'rock').visual = { kind: 'tree' }), /a tree is drawn on a cylinder, not a box/],
    'a non-unit attitude': [(es) => (entity(es, 'goal').pose.attitude = [1, 0.5, 0, 0]), /its attitude is not a unit quaternion/],
    "a part's non-unit attitude": [
      (es) => (entity(es, 'gate').shape.parts[0].offset.attitude = [0.5, 0, 0, 0]),
      /a part's attitude is not a unit quaternion/,
    ],
    'a pose channel that is not a channel': [
      (es) => (entity(es, 'setpoint').pose.position = 'intruder_position'),
      /its position channel intruder_position is not a channel/,
    ],
    'a position channel of width 4': [
      (es) => (entity(es, 'setpoint').pose.position = 'target_attitude'),
      /its position channel target_attitude has width 4, not 3/,
    ],
    'an attitude channel of width 3': [
      (es) => (entity(es, 'setpoint').pose.attitude = 'setpoint_position'),
      /its attitude channel setpoint_position has width 3, not 4/,
    ],
  };
  for (const [name, [edit, expected]] of Object.entries(cases)) {
    const errors = errorsAfter(edit);
    assert.ok(errors.some((e) => expected.test(e)), `${name}: ${errors.join('; ')}`);
  }
});
