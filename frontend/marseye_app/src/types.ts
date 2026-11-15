export type RC = [number, number]
export type Profile = 'rover' | 'astronaut'
export type RouteType = 'unconstrained_reference' | 'shortest_feasible' | 'risk_aware'
export type Layer = 'elevation' | 'slope' | 'roughness' | 'optical' | 'thermal'
export type SelectionMode = 'inspect' | 'start' | 'goal' | 'science'

export interface TerrainGrid {
  grid_shape: [number, number]
  dx_m: number
  dy_m: number
  dataset_hash: string
  elevation_m: (number | null)[][]
  slope_deg: (number | null)[][]
  roughness_m: (number | null)[][]
  valid_mask: boolean[][]
}

export interface Manifest {
  dataset_hash: string
  grid_shape: [number, number]
  bounds_degrees: { west_lon: number; east_lon: number; south_lat: number; north_lat: number }
  topographic_source: string
  optical: { instrument: string; aligned_valid_cells: number; source_shape: [number, number] }
  thermal: {
    instrument: string
    source_product_id: string
    source_resolution_m: [number, number]
    mars_local_time: string
    aligned_valid_cells: number
    stats_K: { min: number; mean: number; max: number }
  }
}

export interface TerrainMetadata {
  dataset_hash: string
  grid_shape: [number, number]
  dx_m: number
  dy_m: number
  profile_names: string[]
  profile_parameters: Record<Profile, { max_cell_slope_deg: number; max_edge_grade_deg: number; slope_weight: number; roughness_weight: number; ascent_weight: number }>
  risk_sources: Record<Profile, string>
}

export interface RouteRecord {
  status: 'found' | 'no_path' | 'blocked_endpoint'
  points_rc: RC[]
  analysis_only?: boolean
  distance_m?: number
  mean_slope_deg?: number
  max_slope_deg?: number
  max_edge_grade_deg?: number
  elevation_gain_m?: number
  elevation_loss_m?: number
  mean_roughness_m?: number
  mean_risk_score_0_1?: number | null
  weighted_cost_m_equiv?: number
  runtime_ms?: number
  risk_source?: string
}

export interface RouteResponse {
  schema_version: string
  terrain_id: string
  start_rc: RC
  goal_rc: RC
  profile: Profile
  risk_source: string
  experimental: boolean
  profile_parameters: { max_cell_slope_deg: number; max_edge_grade_deg: number; slope_weight: number; roughness_weight: number; ascent_weight: number }
  routes: Record<RouteType, RouteRecord>
}

export interface ScienceTarget {
  id: string
  rc: RC
  name: string
  description: string
  reason: string
}

export interface MissionLeg extends RouteRecord {
  from: string
  to: string
  objective_cost?: number
  objective_unit?: string
}

export interface MissionResponse {
  status: 'found' | 'no_path' | 'blocked_endpoint'
  reason?: string
  profile: Profile
  route_type: 'shortest_feasible' | 'risk_aware'
  mode: 'manual' | 'optimized'
  visit_order: string[]
  legs: MissionLeg[]
  points_rc: RC[]
  total_distance_m: number | null
  total_weighted_cost_m_equiv: number | null
  objective_cost: number | null
  objective_unit: string
  risk_source: string
  metrics?: { elevation_gain_m: number; elevation_loss_m: number; max_slope_deg: number; max_edge_grade_deg: number; runtime_ms: number }
}

export interface PointInfo {
  row: number
  col: number
  latitude_deg: number
  longitude_deg_east: number
  valid_dem: boolean
  elevation_m: number | null
  slope_deg: number | null
  roughness_m: number | null
  optical: { intensity: number; instrument: string; unit: string } | null
  thermal: {
    brightness_temperature_K: number
    brightness_temperature_C: number
    instrument: string
    source_resolution_m: number
    mars_local_time: string
  } | null
}
