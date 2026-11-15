import { useEffect, useRef } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import type { Layer, RC, RouteResponse, RouteType, ScienceTarget, TerrainGrid, MissionResponse } from './types'
import { gridToWorldXZ, worldToGridRC } from './terrainMath'

const ROUTE_COLORS: Record<RouteType, number> = {
  unconstrained_reference: 0x65caff,
  shortest_feasible: 0x5edccb,
  risk_aware: 0xba91ff,
}

interface Props {
  terrain: TerrainGrid
  layer: Layer
  opacity: number
  route: RouteResponse | null
  mission: MissionResponse | null
  visibleRoutes: Record<RouteType, boolean>
  start: RC | null
  goal: RC | null
  targets: ScienceTarget[]
  onPick: (point: RC) => void
}

function colorFor(value: number | null, low: number, high: number, palette: string[]) {
  if (value === null) return new THREE.Color('#26323c')
  const t = THREE.MathUtils.clamp((value - low) / Math.max(high - low, 0.001), 0, 1)
  const scaled = t * (palette.length - 1)
  const i = Math.min(Math.floor(scaled), palette.length - 2)
  return new THREE.Color(palette[i]).lerp(new THREE.Color(palette[i + 1]), scaled - i)
}

function labelSprite(label: string, color: string) {
  const canvas = document.createElement('canvas')
  canvas.width = 128
  canvas.height = 128
  const context = canvas.getContext('2d')!
  context.fillStyle = color
  context.beginPath()
  context.arc(64, 64, 45, 0, Math.PI * 2)
  context.fill()
  context.fillStyle = '#07131e'
  context.font = 'bold 42px system-ui'
  context.textAlign = 'center'
  context.textBaseline = 'middle'
  context.fillText(label, 64, 66)
  const texture = new THREE.CanvasTexture(canvas)
  texture.colorSpace = THREE.SRGBColorSpace
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false, toneMapped: false }))
  sprite.scale.set(43, 43, 1)
  sprite.renderOrder = 10
  return sprite
}

export function TerrainView({ terrain, layer, opacity, route, mission, visibleRoutes, start, goal, targets, onPick }: Props) {
  const containerRef = useRef<HTMLDivElement>(null)
  const sceneRef = useRef<THREE.Scene | null>(null)
  const meshRef = useRef<THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial> | null>(null)
  const materialRef = useRef<THREE.MeshStandardMaterial | null>(null)
  const overlayRef = useRef<THREE.Group | null>(null)
  const controlsRef = useRef<OrbitControls | null>(null)
  const onPickRef = useRef(onPick)
  useEffect(() => { onPickRef.current = onPick }, [onPick])

  useEffect(() => {
    const container = containerRef.current
    if (!container) return
    const [rows, cols] = terrain.grid_shape
    const valid = terrain.valid_mask
    const elevations = terrain.elevation_m.flat().filter((v): v is number => v !== null)
    const base = Math.min(...elevations)
    const top = Math.max(...elevations)
    const width = (cols - 1) * terrain.dx_m
    const depth = (rows - 1) * terrain.dy_m
    const verticalScale = Math.min(2.5, Math.max(1, Math.max(width, depth) / (Math.max(top - base, 1) * 2.9)))
    const positions = new Float32Array(rows * cols * 3)
    const uv = new Float32Array(rows * cols * 2)
    const indices: number[] = []
    for (let row = 0; row < rows; row++) {
      for (let col = 0; col < cols; col++) {
        const index = row * cols + col
        const elevation = terrain.elevation_m[row][col] ?? base
        const [x, z] = gridToWorldXZ([row, col], terrain.dx_m, terrain.dy_m)
        positions.set([x, (elevation - base) * verticalScale, z], index * 3)
        uv.set([col / (cols - 1), 1 - row / (rows - 1)], index * 2)
        if (row < rows - 1 && col < cols - 1) {
          const a = index, b = index + 1, d = index + cols, e = d + 1
          if (valid[row][col] && valid[row + 1][col] && valid[row][col + 1]) indices.push(a, d, b)
          if (valid[row][col + 1] && valid[row + 1][col] && valid[row + 1][col + 1]) indices.push(b, d, e)
        }
      }
    }
    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    geometry.setAttribute('uv', new THREE.BufferAttribute(uv, 2))
    geometry.setIndex(indices)
    geometry.computeVertexNormals()

    const scene = new THREE.Scene()
    scene.background = new THREE.Color('#0a1420')
    scene.fog = new THREE.FogExp2('#0a1420', 0.00055)
    const material = new THREE.MeshStandardMaterial({ color: '#ffffff', roughness: 0.93, side: THREE.DoubleSide })
    const mesh = new THREE.Mesh(geometry, material)
    scene.add(mesh)
    scene.add(new THREE.HemisphereLight(0xb7d6ec, 0x683f2f, 2.25))
    const sun = new THREE.DirectionalLight(0xffc995, 3.1)
    sun.position.set(-width * 0.45, width * 1.5, -depth * 0.7)
    scene.add(sun)
    const overlay = new THREE.Group()
    scene.add(overlay)
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2))
    renderer.outputColorSpace = THREE.SRGBColorSpace
    renderer.toneMapping = THREE.ACESFilmicToneMapping
    renderer.toneMappingExposure = 1.42
    container.appendChild(renderer.domElement)
    const camera = new THREE.PerspectiveCamera(44, 1, 0.5, 10000)
    camera.position.set(width * 0.72, width * 1.12, depth * 1.58)
    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.07
    controls.target.set(width / 2, (top - base) * verticalScale * 0.3, depth / 2)
    controls.maxDistance = Math.max(width, depth) * 3.8
    controls.minDistance = 20
    controls.update()
    const resize = () => {
      const rect = container.getBoundingClientRect()
      if (rect.width < 1 || rect.height < 1) return
      camera.aspect = rect.width / rect.height
      camera.updateProjectionMatrix()
      renderer.setSize(rect.width, rect.height, false)
    }
    const observer = new ResizeObserver(resize)
    observer.observe(container)
    resize()

    const raycaster = new THREE.Raycaster()
    const pointer = new THREE.Vector2()
    let down: { x: number; y: number } | null = null
    const pointerDown = (event: PointerEvent) => { down = { x: event.clientX, y: event.clientY } }
    const pointerUp = (event: PointerEvent) => {
      if (!down || Math.hypot(event.clientX - down.x, event.clientY - down.y) > 5) return
      const rect = renderer.domElement.getBoundingClientRect()
      pointer.set((event.clientX - rect.left) / rect.width * 2 - 1, -((event.clientY - rect.top) / rect.height * 2 - 1))
      raycaster.setFromCamera(pointer, camera)
      const hit = raycaster.intersectObject(mesh, false)[0]
      if (!hit) return
      const rc = worldToGridRC(hit.point.x, hit.point.z, terrain.dx_m, terrain.dy_m, rows, cols)
      if (rc && valid[rc[0]][rc[1]]) onPickRef.current(rc)
    }
    renderer.domElement.addEventListener('pointerdown', pointerDown)
    renderer.domElement.addEventListener('pointerup', pointerUp)
    let frame = 0
    const animate = () => {
      frame = requestAnimationFrame(animate)
      controls.update()
      for (const child of overlay.children) {
        if (child.userData.drawStart && child instanceof THREE.Mesh) {
          const total = child.geometry.index?.count || 0
          const progress = Math.min(1, (performance.now() - child.userData.drawStart) / 950)
          child.geometry.setDrawRange(0, Math.floor(total * progress / 6) * 6)
          if (progress >= 1) delete child.userData.drawStart
        }
      }
      renderer.render(scene, camera)
    }
    animate()
    sceneRef.current = scene
    meshRef.current = mesh
    materialRef.current = material
    overlayRef.current = overlay
    controlsRef.current = controls
    return () => {
      cancelAnimationFrame(frame)
      observer.disconnect()
      renderer.domElement.removeEventListener('pointerdown', pointerDown)
      renderer.domElement.removeEventListener('pointerup', pointerUp)
      controls.dispose()
      geometry.dispose()
      material.map?.dispose()
      material.dispose()
      renderer.dispose()
      renderer.domElement.remove()
      sceneRef.current = null
      meshRef.current = null
      materialRef.current = null
      overlayRef.current = null
      controlsRef.current = null
    }
  }, [terrain])

  useEffect(() => {
    const material = materialRef.current
    if (!material) return
    const [rows, cols] = terrain.grid_shape
    const canvas = document.createElement('canvas')
    canvas.width = cols
    canvas.height = rows
    const context = canvas.getContext('2d')!
    const pixels = context.createImageData(cols, rows)
    const elevations = terrain.elevation_m.flat().filter((value): value is number => value !== null)
    const low = Math.min(...elevations), high = Math.max(...elevations)
    for (let row = 0; row < rows; row++) for (let col = 0; col < cols; col++) {
      const value = layer === 'slope' ? terrain.slope_deg[row][col] : layer === 'roughness' ? terrain.roughness_m[row][col] : terrain.elevation_m[row][col]
      const color = layer === 'slope'
        ? colorFor(value, 0, 30, ['#3e8378', '#c8ae75', '#d97952', '#a93e4c'])
        : layer === 'roughness'
          ? colorFor(value, 0, 0.8, ['#5d9fa1', '#b89e73', '#c05a4e'])
          : colorFor(value, low, high, ['#694c42', '#a16b50', '#c88b60', '#e2b382'])
      color.convertLinearToSRGB()
      const i = (row * cols + col) * 4
      pixels.data[i] = color.r * 255
      pixels.data[i + 1] = color.g * 255
      pixels.data[i + 2] = color.b * 255
      pixels.data[i + 3] = terrain.valid_mask[row][col] ? 255 : 0
    }
    context.putImageData(pixels, 0, 0)
    let cancelled = false
    const finish = () => {
      if (cancelled) return
      const texture = new THREE.CanvasTexture(canvas)
      texture.colorSpace = THREE.SRGBColorSpace
      material.map?.dispose()
      material.map = texture
      material.needsUpdate = true
    }
    if (layer === 'optical' || layer === 'thermal') {
      const image = new Image()
      image.onload = () => {
        context.globalAlpha = opacity
        context.drawImage(image, 0, 0, cols, rows)
        finish()
      }
      image.onerror = finish
      image.src = `/data/${layer}.png`
    } else finish()
    return () => { cancelled = true }
  }, [terrain, layer, opacity])

  useEffect(() => {
    const overlay = overlayRef.current
    if (!overlay) return
    for (const child of [...overlay.children]) {
      overlay.remove(child)
      if ('geometry' in child && child.geometry instanceof THREE.BufferGeometry) child.geometry.dispose()
      if ('material' in child) {
        const materials = Array.isArray(child.material) ? child.material : [child.material]
        for (const material of materials) if (material instanceof THREE.Material) material.dispose()
      }
    }
    const elevations = terrain.elevation_m.flat().filter((v): v is number => v !== null)
    const base = Math.min(...elevations)
    const top = Math.max(...elevations)
    const width = (terrain.grid_shape[1] - 1) * terrain.dx_m
    const depth = (terrain.grid_shape[0] - 1) * terrain.dy_m
    const scale = Math.min(2.5, Math.max(1, Math.max(width, depth) / (Math.max(top - base, 1) * 2.9)))
    const position = (rc: RC, lift: number) => {
      const [row, col] = rc
      const [x, z] = gridToWorldXZ(rc, terrain.dx_m, terrain.dy_m)
      return new THREE.Vector3(x, ((terrain.elevation_m[row]?.[col] ?? base) - base) * scale + lift, z)
    }
    const addPath = (points: RC[] | undefined, color: number, lift: number, alpha = 1) => {
      if (!points || points.length < 2) return
      const vectors = points.filter(([r, c]) => r >= 0 && c >= 0 && r < terrain.grid_shape[0] && c < terrain.grid_shape[1] && terrain.valid_mask[r][c]).map(point => position(point, lift))
      if (vectors.length < 2) return
      const curve = new THREE.CurvePath<THREE.Vector3>()
      for (let i = 1; i < vectors.length; i++) curve.add(new THREE.LineCurve3(vectors[i - 1], vectors[i]))
      const tube = new THREE.Mesh(new THREE.TubeGeometry(curve, Math.max(vectors.length * 2, 24), 2.8, 5, false),
        new THREE.MeshBasicMaterial({ color, transparent: true, opacity: alpha, depthTest: false, toneMapped: false }))
      tube.renderOrder = 5
      tube.userData.drawStart = performance.now()
      tube.geometry.setDrawRange(0, 0)
      overlay.add(tube)
    }
    if (route) for (const type of Object.keys(ROUTE_COLORS) as RouteType[]) {
      if (visibleRoutes[type] && route.routes[type]?.status === 'found') addPath(route.routes[type].points_rc, ROUTE_COLORS[type], type === 'unconstrained_reference' ? 5 : type === 'shortest_feasible' ? 7 : 9, type === 'unconstrained_reference' ? 0.8 : 1)
    }
    if (mission?.status === 'found') mission.legs.forEach((leg, index) => addPath(leg.points_rc, index % 2 ? 0xffca89 : 0xffa865, 13, 0.95))
    const addMarker = (rc: RC | null, label: string, color: string) => {
      if (!rc) return
      const marker = labelSprite(label, color)
      marker.position.copy(position(rc, 26))
      overlay.add(marker)
      const ring = new THREE.Mesh(new THREE.TorusGeometry(9, 2, 8, 36), new THREE.MeshBasicMaterial({ color, depthTest: false, toneMapped: false }))
      ring.rotation.x = Math.PI / 2
      ring.position.copy(position(rc, 3))
      ring.renderOrder = 7
      overlay.add(ring)
    }
    addMarker(start, 'A', '#6ce2d1')
    addMarker(goal, 'B', '#ffbb80')
    targets.forEach((target, index) => addMarker(target.rc, `S${index + 1}`, '#caa2ff'))
  }, [terrain, route, mission, visibleRoutes, start, goal, targets])

  return <div className="terrain-canvas" ref={containerRef} aria-label="Interactive 3D Gale terrain" />
}
