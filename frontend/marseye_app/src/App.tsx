import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { TerrainView } from './TerrainView'
import { copy, type CopyKey, type Locale } from './i18n'
import type { Layer, Manifest, MissionResponse, PointInfo, Profile, RC, RouteRecord, RouteResponse, RouteType, ScienceTarget, SelectionMode, TerrainGrid, TerrainMetadata } from './types'

const API = import.meta.env.VITE_API_BASE || 'http://127.0.0.1:8001'
const ROUTE_TYPES: RouteType[] = ['unconstrained_reference', 'shortest_feasible', 'risk_aware']
const ROUTE_LABEL: Record<RouteType, CopyKey> = { unconstrained_reference: 'reference', shortest_feasible: 'feasible', risk_aware: 'riskAware' }
const ROUTE_COLORS: Record<RouteType, string> = { unconstrained_reference: '#65caff', shortest_feasible: '#5edccb', risk_aware: '#ba91ff' }
const LAYERS: { key: Layer; label: CopyKey; icon: string }[] = [
  { key: 'elevation', label: 'elevation', icon: '◩' },
  { key: 'slope', label: 'slope', icon: '◭' },
  { key: 'roughness', label: 'rough', icon: '◌' },
  { key: 'optical', label: 'optical', icon: '◎' },
  { key: 'thermal', label: 'thermal', icon: '◉' },
]

async function getJson<T>(url: string, options?: RequestInit): Promise<T> {
  const response = await fetch(url, options)
  if (!response.ok) {
    const body = await response.text()
    throw new Error(`${response.status}: ${body.slice(0, 180)}`)
  }
  return response.json() as Promise<T>
}

function samePoint(a: RC | null, b: RC | null) {
  return !!a && !!b && a[0] === b[0] && a[1] === b[1]
}

export default function App() {
  const [locale, setLocale] = useState<Locale>(() => (localStorage.getItem('marseye-locale') === 'en' ? 'en' : 'tr'))
  const t = (key: CopyKey) => copy[locale][key]
  const [tab, setTab] = useState<'navigation' | 'science' | 'region'>('navigation')
  const [terrain, setTerrain] = useState<TerrainGrid | null>(null)
  const [manifest, setManifest] = useState<Manifest | null>(null)
  const [metadata, setMetadata] = useState<TerrainMetadata | null>(null)
  const [startupError, setStartupError] = useState<string | null>(null)
  const [apiError, setApiError] = useState<string | null>(null)
  const [mode, setMode] = useState<SelectionMode>('inspect')
  const [profile, setProfile] = useState<Profile>('rover')
  const [layer, setLayer] = useState<Layer>('optical')
  const [opacity, setOpacity] = useState(0.68)
  const [start, setStart] = useState<RC | null>(null)
  const [goal, setGoal] = useState<RC | null>(null)
  const [targets, setTargets] = useState<ScienceTarget[]>([])
  const [selected, setSelected] = useState<RC | null>(null)
  const [point, setPoint] = useState<PointInfo | null>(null)
  const [route, setRoute] = useState<RouteResponse | null>(null)
  const [mission, setMission] = useState<MissionResponse | null>(null)
  const [routeType, setRouteType] = useState<RouteType>('shortest_feasible')
  const [missionType, setMissionType] = useState<'shortest_feasible' | 'risk_aware'>('risk_aware')
  const [missionMode, setMissionMode] = useState<'manual' | 'optimized'>('manual')
  const [visibleRoutes, setVisibleRoutes] = useState<Record<RouteType, boolean>>({ unconstrained_reference: false, shortest_feasible: true, risk_aware: true })
  const [routeBusy, setRouteBusy] = useState(false)
  const [missionBusy, setMissionBusy] = useState(false)
  const [routeMessage, setRouteMessage] = useState<string | null>(null)
  const [missionMessage, setMissionMessage] = useState<string | null>(null)
  const [autoRoute, setAutoRoute] = useState(false)
  const [autoMission, setAutoMission] = useState(false)
  const [viewKey, setViewKey] = useState(0)
  const [showHelp, setShowHelp] = useState(false)
  const routeAbort = useRef<AbortController | null>(null)
  const missionAbort = useRef<AbortController | null>(null)
  const pointAbort = useRef<AbortController | null>(null)

  useEffect(() => {
    document.documentElement.lang = locale
    localStorage.setItem('marseye-locale', locale)
  }, [locale])

  useEffect(() => {
    let active = true
    async function load() {
      try {
        const [grid, source] = await Promise.all([
          getJson<TerrainGrid>('/data/terrain.json'), getJson<Manifest>('/data/manifest.json'),
        ])
        if (grid.dataset_hash !== source.dataset_hash || grid.grid_shape.join() !== source.grid_shape.join()) throw new Error('Terrain asset version mismatch')
        if (!active) return
        setTerrain(grid)
        setManifest(source)
        try {
          const live = await getJson<TerrainMetadata>(`${API}/api/terrain/metadata`)
          if (live.dataset_hash !== grid.dataset_hash) throw new Error('Live API uses a different DEM version')
          if (active) { setMetadata(live); setApiError(null) }
        } catch (error) { if (active) setApiError(String(error)) }
      } catch (error) { if (active) setStartupError(String(error)) }
    }
    void load()
    return () => { active = false }
  }, [])

  useEffect(() => {
    pointAbort.current?.abort()
    if (!selected || !metadata) { setPoint(null); return }
    const controller = new AbortController()
    pointAbort.current = controller
    void getJson<PointInfo>(`${API}/api/point?row=${selected[0]}&col=${selected[1]}`, { signal: controller.signal })
      .then(setPoint).catch(() => { if (!controller.signal.aborted) setPoint(null) })
    return () => controller.abort()
  }, [selected, metadata])

  const invalidate = useCallback(() => {
    routeAbort.current?.abort()
    missionAbort.current?.abort()
    setRoute(null)
    setMission(null)
    setAutoRoute(false)
    setAutoMission(false)
    setRouteBusy(false)
    setMissionBusy(false)
    setRouteMessage(null)
    setMissionMessage(null)
  }, [])

  const pick = useCallback((rc: RC) => {
    setSelected(rc)
    if (mode === 'start') { invalidate(); setStart(rc) }
    else if (mode === 'goal') { invalidate(); setGoal(rc) }
    else if (mode === 'science') {
      if (targets.length >= 5) { setMissionMessage(locale === 'tr' ? 'En fazla beş bilim noktası eklenebilir.' : 'Add up to five science points.'); return }
      if (samePoint(start, rc) || samePoint(goal, rc) || targets.some(target => samePoint(target.rc, rc))) {
        setMissionMessage(locale === 'tr' ? 'Bu hücre zaten seçili.' : 'This cell is already selected.'); return
      }
      invalidate()
      const next = Array.from({ length: 5 }, (_, i) => `S${i + 1}`).find(id => !targets.some(target => target.id === id))!
      setTargets([...targets, { id: next, rc, name: next, description: '', reason: '' }])
      setTab('science')
    }
    setMode('inspect')
  }, [mode, targets, start, goal, invalidate, locale])

  const calculateRoute = useCallback(async () => {
    if (!start || !goal || !metadata) return
    routeAbort.current?.abort()
    const controller = new AbortController()
    routeAbort.current = controller
    setRouteBusy(true)
    setRouteMessage(null)
    setRoute(null)
    try {
      const result = await getJson<RouteResponse>(`${API}/api/route`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, signal: controller.signal,
        body: JSON.stringify({ start_rc: start, goal_rc: goal, profile, include_reference: true }),
      })
      if (!controller.signal.aborted) {
        setRoute(result)
        if (result.routes.shortest_feasible.status !== 'found' && result.routes.risk_aware.status !== 'found')
          setRouteMessage(result.routes.shortest_feasible.status === 'blocked_endpoint' ? copy[locale].blocked : copy[locale].noPath)
      }
    } catch (error) { if (!controller.signal.aborted) setRouteMessage(String(error)) }
    finally { if (!controller.signal.aborted) setRouteBusy(false) }
  }, [start, goal, profile, metadata, locale])

  useEffect(() => { if (autoRoute) void calculateRoute() }, [autoRoute, calculateRoute])

  const calculateMission = useCallback(async () => {
    if (!start || !goal || !metadata || targets.length === 0) return
    missionAbort.current?.abort()
    const controller = new AbortController()
    missionAbort.current = controller
    setMissionBusy(true)
    setMission(null)
    setMissionMessage(null)
    try {
      const result = await getJson<MissionResponse>(`${API}/api/mission`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, signal: controller.signal,
        body: JSON.stringify({ start_rc: start, goal_rc: goal,
          science_targets: targets.map(({ id, rc }) => ({ id, rc })), profile,
          route_type: missionType, mode: missionMode, manual_order: missionMode === 'manual' ? targets.map(target => target.id) : undefined }),
      })
      if (!controller.signal.aborted) {
        setMission(result)
        if (result.status !== 'found') setMissionMessage(result.reason || (result.status === 'blocked_endpoint' ? copy[locale].blocked : copy[locale].noPath))
      }
    } catch (error) { if (!controller.signal.aborted) setMissionMessage(String(error)) }
    finally { if (!controller.signal.aborted) setMissionBusy(false) }
  }, [start, goal, targets, profile, missionType, missionMode, metadata, locale])

  useEffect(() => { if (autoMission) void calculateMission() }, [autoMission, calculateMission])

  const fmt = (value: number | null | undefined, digits = 2) => value == null || !Number.isFinite(value)
    ? '—' : value.toLocaleString(locale === 'tr' ? 'tr-TR' : 'en-US', { maximumFractionDigits: digits, minimumFractionDigits: digits })
  const rcText = (rc: RC | null) => rc ? `[${rc[0]}, ${rc[1]}]` : t('notSelected')
  const metric = (label: CopyKey, value: number | null | undefined, unit: string, digits = 2) => <div className="metric-row"><span>{t(label)}</span><strong>{value == null ? '—' : `${fmt(value, digits)} ${unit}`}</strong></div>
  const selectedRoute: RouteRecord | null = route?.routes[routeType] ?? null
  const selectedTarget = targets.find(target => samePoint(target.rc, selected))
  const selectedRole = samePoint(start, selected) ? 'A' : samePoint(goal, selected) ? 'B' : selectedTarget?.id
  const routeReady = !!start && !!goal && !!metadata
  const elevationRange = useMemo(() => {
    const values = terrain?.elevation_m.flat().filter((value): value is number => value !== null) || []
    return values.length ? [Math.min(...values), Math.max(...values)] : null
  }, [terrain])

  return <div className="app-shell">
    <header className="topbar">
      <div className="brand-lockup"><div className="brand-orbit">◌</div><div><div className="brand-title">Mars<span>Eye</span><sup>V4</sup></div><div className="brand-sub">VOYAGER / SPACE APPS 2026</div></div></div>
      <div className="header-divider" />
      <div className="region-heading"><span className="eyebrow">{t('atlas')}</span><strong>Gale Crater <span>·</span> Mars</strong></div>
      <nav className="top-nav" aria-label="Main navigation">
        {(['navigation', 'science', 'region'] as const).map(item => <button key={item} className={tab === item ? 'nav-active' : ''} onClick={() => setTab(item)}>{t(item === 'region' ? 'region' : item)}</button>)}
      </nav>
      <div className="header-actions"><span className={`api-status ${metadata ? 'online' : ''}`} title={apiError || undefined}><i />{metadata ? t('statusReady') : t('statusOffline')}</span><button className="locale-button" onClick={() => setLocale(locale === 'tr' ? 'en' : 'tr')}>{locale.toUpperCase()} ▾</button><button className="icon-button" onClick={() => setShowHelp(true)} aria-label={t('help')}>?</button></div>
    </header>

    <main className="main-grid">
      <aside className="control-panel">
        {tab === 'navigation' && <>
          <div className="panel-heading"><span className="eyebrow">01 / {t('missionControl')}</span><h1>{t('planRoute')}</h1><p>{t('clickTerrain')}</p></div>
          <div className="point-select-grid">
            <button className={`point-select start ${mode === 'start' ? 'selecting' : ''}`} onClick={() => setMode('start')}><span className="point-letter">A</span><span><small>{t('start')}</small><strong>{rcText(start)}</strong></span></button>
            <button className={`point-select goal ${mode === 'goal' ? 'selecting' : ''}`} onClick={() => setMode('goal')}><span className="point-letter">B</span><span><small>{t('goal')}</small><strong>{rcText(goal)}</strong></span></button>
          </div>
          <div className="inline-actions"><button onClick={() => setMode('science')} disabled={targets.length >= 5}>＋ {t('addScience')} <span>{targets.length}/5</span></button><button onClick={() => { invalidate(); setStart(null); setGoal(null); setTargets([]); setSelected(null) }}>{t('clear')}</button></div>
          <div className="section-divider" />
          <section><div className="section-title">{t('mobility')}</div><div className="segmented"><button className={profile === 'rover' ? 'chosen' : ''} onClick={() => setProfile('rover')}>◈ {t('rover')}</button><button className={profile === 'astronaut' ? 'chosen' : ''} onClick={() => setProfile('astronaut')}>✧ {t('astronaut')}</button></div><p className="micro-copy">{t('profilesDiffer')}</p>{metadata && <div className="profile-limits"><span>{locale === 'tr' ? 'Hücre eğimi' : 'Cell slope'} &lt; {fmt(metadata.profile_parameters[profile].max_cell_slope_deg, 0)}°</span><span>{locale === 'tr' ? 'Geçiş eğimi' : 'Edge grade'} &lt; {fmt(metadata.profile_parameters[profile].max_edge_grade_deg, 0)}°</span><span>{locale === 'tr' ? 'Eğim / pürüz / tırmanış ağırlıkları' : 'Slope / roughness / ascent weights'}: {metadata.profile_parameters[profile].slope_weight} / {metadata.profile_parameters[profile].roughness_weight} / {metadata.profile_parameters[profile].ascent_weight}</span></div>}</section>
          <button className="primary-action" disabled={!routeReady || routeBusy} onClick={() => autoRoute ? void calculateRoute() : setAutoRoute(true)}>{routeBusy ? t('loading') : t('calculate')} <span>↗</span></button>
          {routeBusy && <button className="cancel-action" onClick={() => { routeAbort.current?.abort(); setRouteBusy(false); setAutoRoute(false); setRouteMessage(locale === 'tr' ? 'İstek iptal edildi.' : 'Request cancelled.') }}>{t('cancel')}</button>}
          {routeMessage && <div className="alert" role="alert">{routeMessage}</div>}
          <div className="section-divider" />
          <section><div className="section-title">{t('routeComparison')}</div><div className="route-list">{ROUTE_TYPES.map(type => <div key={type} className={`route-item ${routeType === type ? 'route-selected' : ''}`}>
            <button className="route-toggle" type="button" aria-label={`${t(ROUTE_LABEL[type])} visibility`} aria-pressed={visibleRoutes[type]} onClick={() => setVisibleRoutes(previous => ({ ...previous, [type]: !previous[type] }))}><i style={{ backgroundColor: ROUTE_COLORS[type], opacity: visibleRoutes[type] ? 1 : .25 }} /></button>
            <button className="route-main" onClick={() => setRouteType(type)}><span><strong>{t(ROUTE_LABEL[type])}</strong><small>{type === 'unconstrained_reference' ? t('analysisOnly') : route?.routes[type]?.status === 'found' ? `${fmt(route.routes[type].distance_m)} m` : t('unavailable')}</small></span><span>›</span></button>
          </div>)}</div></section>
          <p className="caveat">{t('sourceCaveat')}</p>
        </>}

        {tab === 'science' && <>
          <div className="panel-heading"><span className="eyebrow">02 / {t('scienceTargets')}</span><h1>{t('science')}</h1><p>{t('proposed')}</p></div>
          <button className={`secondary-action ${mode === 'science' ? 'selecting' : ''}`} disabled={targets.length >= 5} onClick={() => setMode('science')}>＋ {t('addScience')} <span>{targets.length}/5</span></button>
          <div className="target-list">{targets.map((target, index) => <div className="target-card" key={target.id}>
            <div className="target-header"><span className="target-badge">{index + 1}</span><strong>{target.id} · {rcText(target.rc)}</strong><button onClick={() => { invalidate(); setTargets(targets.filter(item => item.id !== target.id)) }} aria-label={`${t('clear')} ${target.id}`}>×</button></div>
            <label>{t('name')}<input value={target.name} maxLength={60} onChange={event => setTargets(targets.map(item => item.id === target.id ? { ...item, name: event.target.value } : item))} /></label>
            <label>{t('description')}<input value={target.description} maxLength={160} onChange={event => setTargets(targets.map(item => item.id === target.id ? { ...item, description: event.target.value } : item))} /></label>
            <label>{t('reason')}<input value={target.reason} maxLength={160} onChange={event => setTargets(targets.map(item => item.id === target.id ? { ...item, reason: event.target.value } : item))} /></label>
            <div className="order-actions"><button disabled={index === 0} onClick={() => { invalidate(); const next = [...targets]; [next[index - 1], next[index]] = [next[index], next[index - 1]]; setTargets(next) }}>↑</button><button disabled={index === targets.length - 1} onClick={() => { invalidate(); const next = [...targets]; [next[index + 1], next[index]] = [next[index], next[index + 1]]; setTargets(next) }}>↓</button></div>
          </div>)}</div>
          {targets.length === 0 && <div className="empty-state">{t('emptyMission')}</div>}
          <div className="section-divider" />
          <section><div className="section-title">{t('missionRoute')}</div><div className="segmented"><button className={missionMode === 'manual' ? 'chosen' : ''} onClick={() => setMissionMode('manual')}>{t('manual')}</button><button className={missionMode === 'optimized' ? 'chosen' : ''} onClick={() => setMissionMode('optimized')}>{t('optimized')}</button></div>
            <div className="segmented secondary"><button className={missionType === 'shortest_feasible' ? 'chosen' : ''} onClick={() => setMissionType('shortest_feasible')}>{t('feasible')}</button><button className={missionType === 'risk_aware' ? 'chosen' : ''} onClick={() => setMissionType('risk_aware')}>{t('riskAware')}</button></div></section>
          <button className="primary-action" disabled={!routeReady || targets.length === 0 || missionBusy} onClick={() => autoMission ? void calculateMission() : setAutoMission(true)}>{missionBusy ? t('loading') : t('planMission')} <span>↗</span></button>
          {missionBusy && <button className="cancel-action" onClick={() => { missionAbort.current?.abort(); setMissionBusy(false); setAutoMission(false) }}>{t('cancel')}</button>}
          {missionMessage && <div className="alert" role="alert">{missionMessage}</div>}
          <p className="caveat">{t('scienceDisclaimer')}</p>
        </>}

        {tab === 'region' && <>
          <div className="panel-heading"><span className="eyebrow">03 / {t('regionInfo')}</span><h1>Gale Crater</h1><p>{t('pilot')}</p></div>
          <div className="region-card"><span>137.5280067° E</span><span>−4.6926009°</span><strong>GALE / MARS</strong></div>
          <p className="info-paragraph">{locale === 'tr'
            ? 'Gale, yaklaşık 154 km çapında bir Mars krateridir. Curiosity gezgini, Mars Science Laboratory görevi kapsamında krater içindeki katmanlı arazileri inceler. Bu atlas yalnızca seçilmiş yaklaşık 1 km² pilot alanı gösterir.'
            : 'Gale is a Martian crater about 154 km across. Curiosity explores its layered terrain as part of the Mars Science Laboratory mission. This atlas covers only a selected ~1 km² pilot area.'}
            {' '}<a href="https://science.nasa.gov/resource/context-of-curiosity-landing-site-in-gale-crater/" target="_blank" rel="noreferrer">NASA ↗</a>
            {' · '}<a href="https://planetarynames.wr.usgs.gov/Feature/2071" target="_blank" rel="noreferrer">USGS ↗</a></p>
          <div className="section-title">{t('coordinates')}</div>
          <div className="metric-row"><span>West / East</span><strong>{fmt(manifest?.bounds_degrees.west_lon, 5)}° / {fmt(manifest?.bounds_degrees.east_lon, 5)}°</strong></div>
          <div className="metric-row"><span>South / North</span><strong>{fmt(manifest?.bounds_degrees.south_lat, 5)}° / {fmt(manifest?.bounds_degrees.north_lat, 5)}°</strong></div>
          <div className="metric-row"><span>DEM min / max</span><strong>{fmt(elevationRange?.[0], 1)} / {fmt(elevationRange?.[1], 1)} m</strong></div>
          <div className="section-divider" /><div className="section-title">{t('dataOrigin')}</div>
          <p className="info-paragraph">{manifest?.topographic_source || t('sourceUnknown')}</p>
          <p className="info-paragraph">MRO CTX · {manifest?.optical.source_shape.join(' × ')} px · <a href="https://science.nasa.gov/resource/before-and-after-curiositys-touchdown/" target="_blank" rel="noreferrer">NASA instrument context ↗</a></p>
          <p className="info-paragraph">THEMIS {manifest?.thermal.source_product_id} · ~100 m/px · 03:15 Mars local time · <a href="https://themis.asu.edu/faq" target="_blank" rel="noreferrer">ASU THEMIS ↗</a></p>
          <div className="section-divider" /><div className="section-title">{t('limitation')}</div>
          <p className="micro-copy">{t('sourceCaveat')}</p><p className="micro-copy">{t('opticalCaveat')}</p><p className="micro-copy">{t('thermalCaveat')}</p>
          <p className="info-paragraph">NASA Mars Science Laboratory / Curiosity: <a href="https://science.nasa.gov/mission/msl-curiosity/" target="_blank" rel="noreferrer">NASA Science ↗</a></p>
        </>}
      </aside>

      <section className="map-stage" aria-label="Mars 3D exploration map">
        {terrain ? <TerrainView key={viewKey} terrain={terrain} layer={layer} opacity={opacity} route={route} mission={mission} visibleRoutes={visibleRoutes} start={start} goal={goal} targets={targets} onPick={pick} /> : <div className="map-loading">{startupError || (locale === 'tr' ? 'Gale arazi verisi yükleniyor…' : 'Loading Gale terrain…')}</div>}
        <div className="map-vignette" />
        <div className="map-top-left"><span className="map-kicker">MARS / GALE CRATER</span><h2>{t('explore')}<span>.</span></h2><p>{t('pilot')}</p></div>
        <div className="map-top-right"><button onClick={() => setViewKey(viewKey + 1)}>⌖ &nbsp;{t('reset')}</button><button onClick={() => setShowHelp(true)}>ⓘ &nbsp;{t('help')}</button></div>
        <div className="map-bottom-left"><div className="layer-heading"><strong>{t('terrainLayers')}</strong><span>{terrain?.grid_shape.join(' × ')} · {fmt(terrain?.dx_m, 2)} × {fmt(terrain?.dy_m, 2)} m</span></div><div className="layer-options">{LAYERS.map(item => <button key={item.key} className={layer === item.key ? 'active' : ''} onClick={() => setLayer(item.key)} title={t(item.label)}><span>{item.icon}</span><small>{t(item.label)}</small></button>)}</div>{(layer === 'optical' || layer === 'thermal') && <label className="opacity-control">{t('opacity')} <input type="range" min="0" max="1" step="0.05" value={opacity} onChange={event => setOpacity(Number(event.target.value))} /> {Math.round(opacity * 100)}%</label>}{layer === 'thermal' && <div className="gradient-legend"><span>{fmt(manifest?.thermal.stats_K.min, 1)} K / {fmt((manifest?.thermal.stats_K.min ?? 0) - 273.15, 1)}°C</span><i /><span>{fmt(manifest?.thermal.stats_K.max, 1)} K / {fmt((manifest?.thermal.stats_K.max ?? 0) - 273.15, 1)}°C</span></div>}{layer === 'elevation' && <div className="gradient-legend elevation-legend"><span>{fmt(elevationRange?.[0], 0)} m</span><i /><span>{fmt(elevationRange?.[1], 0)} m</span></div>}<p>{layer === 'thermal' ? t('thermalCaveat') : layer === 'optical' ? t('opticalCaveat') : t('mapHint')}</p></div>
        {mode !== 'inspect' && <div className="selection-prompt">◎ &nbsp;{mode === 'start' ? t('selectStart') : mode === 'goal' ? t('selectGoal') : t('addScience')} — {t('clickTerrain')}</div>}
      </section>

      <aside className="analysis-panel"><div className="analysis-heading"><span className="eyebrow">LIVE ANALYSIS / GALE 01</span><h2>{t('pointAnalysis')}</h2></div>
        {point ? <><div className="selected-point"><div className="selected-icon">{selectedRole || '⌖'}</div><div><small>{selectedRole || t('inspect')}</small><strong>{selectedTarget?.name || `[${point.row}, ${point.col}]`}</strong><span>{fmt(point.latitude_deg, 6)}°, {fmt(point.longitude_deg_east, 6)}° E</span></div></div>
          <div className="analysis-block"><div className="section-title">{t('topography')}</div>{metric('elevation', point.elevation_m, 'm')}{metric('slope', point.slope_deg, '°')}{metric('roughness', point.roughness_m, 'm', 3)}</div>
          <div className="analysis-block"><div className="section-title">{t('opticalData')}</div>{point.optical ? <><div className="metric-row"><span>MRO CTX</span><strong>{fmt(point.optical.intensity, 1)}</strong></div><p className="micro-copy">{t('opticalCaveat')}</p></> : <p className="unavailable">{t('noObservation')}</p>}</div>
          <div className="analysis-block"><div className="section-title">{t('thermalData')}</div>{point.thermal ? <>{metric('thermal', point.thermal.brightness_temperature_K, 'K')}{metric('thermal', point.thermal.brightness_temperature_C, '°C')}<p className="micro-copy">THEMIS Band 9 · ~100 m · 03:15</p></> : <p className="unavailable">{t('noObservation')}</p>}</div>
        </> : <div className="analysis-empty"><span>⌖</span><p>{t('pickPoint')}</p></div>}
          <div className="analysis-block"><div className="section-title">{t('routeComparison')}</div>{selectedRoute?.status === 'found' ? <><div className="current-route"><i style={{ background: ROUTE_COLORS[routeType] }} /><div><small>{profile === 'rover' ? t('rover') : t('astronaut')}</small><strong>{t(ROUTE_LABEL[routeType])}</strong></div></div>{routeType === 'unconstrained_reference' && <div className="alert warning">{t('analysisOnly')}</div>}{metric('distance', selectedRoute.distance_m, 'm')}{metric('meanSlope', selectedRoute.mean_slope_deg, '°')}{metric('maxSlope', selectedRoute.max_slope_deg, '°')}{metric('maxGrade', selectedRoute.max_edge_grade_deg, '°')}{metric('gain', selectedRoute.elevation_gain_m, 'm')}{metric('roughness', selectedRoute.mean_roughness_m, 'm', 3)}{metric('cost', routeType === 'unconstrained_reference' ? null : selectedRoute.weighted_cost_m_equiv, 'm-eq')}{metric('runtime', selectedRoute.runtime_ms, 'ms')}{selectedRoute.mean_risk_score_0_1 != null && metric('riskScore', selectedRoute.mean_risk_score_0_1, '', 3)}<p className="micro-copy">{t('noteCost')}</p></> : <p className="unavailable">{t('unavailable')}</p>}</div>
        {mission?.status === 'found' && <div className="analysis-block">
          <div className="section-title">{t('missionRoute')}</div>
          <p className="visit-order">{mission.visit_order.join(' → ')}</p>
          {metric('distance', mission.total_distance_m, 'm')}
          {metric('gain', mission.metrics?.elevation_gain_m, 'm')}
          {metric('maxSlope', mission.metrics?.max_slope_deg, '°')}
          {metric('maxGrade', mission.metrics?.max_edge_grade_deg, '°')}
          {metric('cost', mission.objective_cost, mission.objective_unit === 'm' ? 'm' : 'm-eq')}
          <div className="leg-list">{mission.legs.map((leg, index) => <details key={`${leg.from}-${leg.to}`}>
            <summary><span>0{index + 1} &nbsp;{leg.from} → {leg.to}</span><strong>{fmt(leg.distance_m)} m</strong></summary>
            {metric('gain', leg.elevation_gain_m, 'm')}{metric('maxSlope', leg.max_slope_deg, '°')}{metric('cost', leg.objective_cost, leg.objective_unit === 'm' ? 'm' : 'm-eq')}
          </details>)}</div>
        </div>}
        <div className="analysis-foot"><span className="live-dot" /> {t('sourceCaveat')}</div>
      </aside>
    </main>
    {showHelp && <div className="modal-backdrop" onClick={() => setShowHelp(false)}><div className="help-modal" role="dialog" aria-modal="true" aria-label={t('help')} onClick={event => event.stopPropagation()}><button className="close-modal" onClick={() => setShowHelp(false)}>×</button><span className="eyebrow">MARSEYE / FIELD GUIDE</span><h2>{t('help')}</h2><p>{t('mapHint')}</p><p>{t('clickTerrain')}</p><p>{t('analysisOnly')}</p><p>{t('noteCost')}</p><p>{t('thermalCaveat')}</p></div></div>}
  </div>
}
