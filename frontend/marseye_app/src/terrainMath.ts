import type { RC } from './types'

/** Three.js X/Z horizontal axes, with DEM row increasing southward. */
export function gridToWorldXZ([row, col]: RC, dx: number, dy: number): [number, number] {
  return [col * dx, row * dy]
}

/** Snap a mesh hit to its nearest zero-based DEM vertex; never clamp outside hits. */
export function worldToGridRC(x: number, z: number, dx: number, dy: number,
                              rows: number, cols: number): RC | null {
  if (![x, z, dx, dy].every(Number.isFinite) || dx <= 0 || dy <= 0) return null
  const row = Math.round(z / dy)
  const col = Math.round(x / dx)
  return row >= 0 && row < rows && col >= 0 && col < cols ? [row, col] : null
}
