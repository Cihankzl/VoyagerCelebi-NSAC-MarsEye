import test from 'node:test'
import assert from 'node:assert/strict'
import { gridToWorldXZ, worldToGridRC } from '../src/terrainMath.ts'

test('DEM corners and center round-trip through Three.js X/Z', () => {
  for (const rc of [[0, 0], [0, 199], [197, 0], [197, 199], [99, 100]]) {
    const [x, z] = gridToWorldXZ(rc, 5.0033693, 4.9968054)
    assert.deepEqual(worldToGridRC(x, z, 5.0033693, 4.9968054, 198, 200), rc)
  }
})

test('row increases southward and out-of-grid hits are rejected', () => {
  assert.equal(gridToWorldXZ([1, 0], 5.0033693, 4.9968054)[1] > gridToWorldXZ([0, 0], 5.0033693, 4.9968054)[1], true)
  assert.equal(worldToGridRC(-10, 0, 5, 5, 198, 200), null)
  assert.equal(worldToGridRC(0, 1000, 5, 5, 198, 200), null)
})
