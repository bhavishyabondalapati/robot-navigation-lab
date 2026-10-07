import { useCallback, useEffect, useRef, useState } from 'react'
import { fetchMap, openSimSocket, planOnce } from './api.js'
import ErrorChart from './components/ErrorChart.jsx'
import GridCanvas from './components/GridCanvas.jsx'
import StatsTable from './components/StatsTable.jsx'

const PRESETS = [
  ['warehouse', 'Warehouse'],
  ['maze', 'Maze'],
  ['open_field', 'Open field'],
]
const TOOLS = [
  ['wall', 'Draw wall'],
  ['erase', 'Erase'],
  ['start', 'Move start'],
  ['goal', 'Move goal'],
]
const PLANNERS = [
  ['astar', 'A*'],
  ['rrt', 'RRT'],
]

const statsOf = (p) => ({
  success: p.success,
  path_length: p.path_length,
  nodes_expanded: p.nodes_expanded,
  planning_time_ms: p.planning_time_ms,
})

export default function App() {
  const [preset, setPreset] = useState('warehouse')
  const [grid, setGrid] = useState([[0]])
  const [start, setStart] = useState([1, 1])
  const [goal, setGoal] = useState([2, 2])
  const [tool, setTool] = useState('wall')
  const [planner, setPlanner] = useState('astar')
  const [seed, setSeed] = useState(42)
  const [delay, setDelay] = useState(50)
  const [layers, setLayers] = useState({ explored: true, particles: true, lidar: true })

  const [plan, setPlan] = useState(null)
  const [results, setResults] = useState({}) // planner -> last stats
  const [sim, setSim] = useState(null)
  const [errors, setErrors] = useState([])
  const [paused, setPaused] = useState(false)
  const [message, setMessage] = useState('')
  const wsRef = useRef(null)

  const live = sim !== null && sim.status === 'running'

  const stopSocket = useCallback(() => {
    if (wsRef.current) {
      wsRef.current.onclose = null
      wsRef.current.close()
      wsRef.current = null
    }
  }, [])

  const resetRun = useCallback(() => {
    stopSocket()
    setSim(null)
    setErrors([])
    setPaused(false)
  }, [stopSocket])

  const loadPreset = useCallback(
    async (name) => {
      resetRun()
      try {
        const m = await fetchMap(name)
        setGrid(m.grid)
        setStart(m.start)
        setGoal(m.goal)
        setPlan(null)
        setResults({})
        setMessage('')
      } catch (e) {
        setMessage(`${e.message}. Is the backend running on port 8000?`)
      }
    },
    [resetRun],
  )

  useEffect(() => {
    loadPreset(preset)
  }, [preset, loadPreset])

  useEffect(() => stopSocket, [stopSocket]) // close the socket when the page unmounts

  function send(msg) {
    if (wsRef.current?.readyState === WebSocket.OPEN) wsRef.current.send(JSON.stringify(msg))
  }

  function onPaint([x, y]) {
    const same = (c) => c[0] === x && c[1] === y
    if (tool === 'start' || tool === 'goal') {
      if (sim || grid[y][x] || same(tool === 'start' ? goal : start)) return
      if (tool === 'start') setStart([x, y])
      else setGoal([x, y])
      setPlan(null)
      return
    }
    const value = tool === 'wall' ? 1 : 0
    if (grid[y][x] === value || same(start) || same(goal)) return
    if (value && sim && Math.floor(sim.true_pose[0]) === x && Math.floor(sim.true_pose[1]) === y) return
    setGrid((g) => g.map((row, r) => (r === y ? row.map((v, c) => (c === x ? value : v)) : row)))
    if (live) send({ type: 'set_cell', x, y, value }) // the backend decides whether to replan
    else if (!sim) setPlan(null) // the old plan no longer matches the map
  }

  async function runPlanner(name) {
    const p = await planOnce({ grid, start, goal, planner: name, seed })
    setResults((r) => ({ ...r, [name]: statsOf(p) }))
    return p
  }

  async function onPlan() {
    resetRun()
    try {
      const p = await runPlanner(planner)
      setPlan(p)
      setMessage(p.success ? '' : 'No path: the goal is unreachable.')
    } catch (e) {
      setMessage(e.message)
    }
  }

  async function onCompare() {
    resetRun()
    try {
      const other = planner === 'astar' ? 'rrt' : 'astar'
      await runPlanner(other)
      setPlan(await runPlanner(planner)) // show the selected planner on the map
      setMessage('')
    } catch (e) {
      setMessage(e.message)
    }
  }

  function onRun() {
    resetRun()
    setMessage('')
    const ws = openSimSocket(
      (msg) => {
        if (msg.type === 'plan') {
          setPlan(msg)
          // Only the first plan of a run goes in the comparison table: a replan
          // starts mid-route, so its length isn't comparable.
          if (!msg.replan) setResults((r) => ({ ...r, [msg.planner]: statsOf(msg) }))
          if (msg.replan) setMessage(msg.success ? 'Obstacle detected on the path, replanned.' : 'Replanning failed: no path.')
        } else if (msg.type === 'state') {
          setSim(msg)
          setErrors((e) => (msg.step === 0 ? [msg.error] : [...e, msg.error]))
        } else if (msg.type === 'error') {
          setMessage(msg.message)
        }
      },
      () => setMessage('Connection to the backend closed.'),
    )
    ws.onopen = () => {
      ws.send(JSON.stringify({ type: 'speed', delay_ms: delay }))
      ws.send(JSON.stringify({ type: 'start', grid, start, goal, planner, seed }))
    }
    wsRef.current = ws
  }

  function togglePause() {
    send({ type: paused ? 'resume' : 'pause' })
    setPaused(!paused)
  }

  const statusText = sim
    ? { running: paused ? 'Paused' : 'Running', reached: 'Goal reached', no_path: 'No path', timeout: 'Gave up (too many steps)' }[sim.status]
    : 'Idle'

  return (
    <div className="app">
      <header>
        <h1>Robot Navigation Lab</h1>
        <p className="sub">A* vs RRT path planning · particle filter localization · live replanning</p>
      </header>

      <main>
        <section className="map-panel">
          <div className="toolbar">
            <div className="seg" role="group" aria-label="Edit tool">
              {TOOLS.map(([id, label]) => (
                <button key={id} className={tool === id ? 'on' : ''} onClick={() => setTool(id)}>
                  {label}
                </button>
              ))}
            </div>
            <div className="layer-toggles">
              {[
                ['explored', 'Search'],
                ['particles', 'Particles'],
                ['lidar', 'Lidar'],
              ].map(([k, label]) => (
                <label key={k}>
                  <input type="checkbox" checked={layers[k]} onChange={() => setLayers((l) => ({ ...l, [k]: !l[k] }))} />
                  {label}
                </label>
              ))}
            </div>
          </div>
          <div className="canvas-wrap">
            <GridCanvas grid={grid} start={start} goal={goal} plan={plan} sim={sim} layers={layers} onPaint={onPaint} />
          </div>
          <div className="legend">
            <span><i className="sw path" />Path</span>
            <span><i className="sw explored" />Explored / tree</span>
            <span><i className="dot true" />True pose</span>
            <span><i className="dot est" />Estimate</span>
            <span><i className="dot particle" />Particles</span>
            <span><i className="sw beam" />Lidar</span>
          </div>
        </section>

        <aside>
          <div className="card">
            <h2>Setup</h2>
            <label className="row">
              Map
              <select value={preset} onChange={(e) => setPreset(e.target.value)}>
                {PRESETS.map(([id, label]) => (
                  <option key={id} value={id}>{label}</option>
                ))}
              </select>
            </label>
            <div className="row">
              Planner
              <div className="seg small" role="group" aria-label="Planner">
                {PLANNERS.map(([id, label]) => (
                  <button key={id} className={planner === id ? 'on' : ''} onClick={() => setPlanner(id)} disabled={live}>
                    {label}
                  </button>
                ))}
              </div>
            </div>
            <label className="row">
              Seed
              <input type="number" value={seed} onChange={(e) => setSeed(Number(e.target.value) || 0)} disabled={live} />
            </label>
            <label className="row">
              Step delay
              <input
                type="range" min="5" max="200" value={delay}
                onChange={(e) => {
                  setDelay(Number(e.target.value))
                  send({ type: 'speed', delay_ms: Number(e.target.value) })
                }}
              />
              <span className="mono">{delay} ms</span>
            </label>
            <div className="buttons">
              <button onClick={onPlan} disabled={live}>Plan</button>
              <button onClick={onCompare} disabled={live} title="Plan with both A* and RRT">Compare</button>
              <button className="primary" onClick={onRun} disabled={live && !paused}>Run</button>
              <button onClick={togglePause} disabled={!live}>{paused ? 'Resume' : 'Pause'}</button>
              <button onClick={() => { resetRun(); setPlan(null) }}>Reset</button>
            </div>
            {message && <p className="message">{message}</p>}
          </div>

          <div className="card">
            <h2>Planner comparison</h2>
            <StatsTable results={results} />
          </div>

          <div className="card">
            <h2>Localization</h2>
            <div className="kv">
              <span>Status</span><b>{statusText}</b>
              <span>Step</span><b className="mono">{sim?.step ?? 0}</b>
              <span>Error (cells)</span><b className="mono">{sim ? sim.error.toFixed(3) : '–'}</b>
              <span>Replans</span><b className="mono">{sim?.replans ?? 0}</b>
            </div>
            <p className="chart-title">Position error over time (cells)</p>
            <ErrorChart errors={errors} />
          </div>
        </aside>
      </main>
    </div>
  )
}
