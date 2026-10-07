import { useEffect, useRef } from 'react'

export const CELL = 18 // pixels per grid cell

// Canvas colors. The map is always drawn on a light "floor" so it reads the
// same in light and dark mode.
const C = {
  floor: '#f6f5f1',
  gridLine: 'rgba(0,0,0,0.05)',
  wall: '#2f343c',
  explored: 'rgba(59,130,246,0.20)',
  tree: 'rgba(37,99,235,0.45)',
  path: '#f59e0b',
  start: '#16a34a',
  goal: '#dc2626',
  particle: 'rgba(124,58,237,0.55)',
  beam: 'rgba(220,38,38,0.28)',
  hit: '#dc2626',
  truePose: '#0f766e',
  estPose: '#7c3aed',
}

function drawRobot(ctx, pose, color, filled) {
  const [x, y, th] = pose.map((v, i) => (i < 2 ? v * CELL : v))
  const r = CELL * 0.45
  ctx.beginPath()
  ctx.arc(x, y, r, 0, Math.PI * 2)
  if (filled) {
    ctx.fillStyle = color
    ctx.fill()
  }
  ctx.lineWidth = 2.5
  ctx.strokeStyle = filled ? '#ffffff' : color
  ctx.stroke()
  ctx.beginPath()
  ctx.moveTo(x, y)
  ctx.lineTo(x + Math.cos(th) * r * 1.6, y + Math.sin(th) * r * 1.6)
  ctx.strokeStyle = color
  ctx.lineWidth = 2.5
  ctx.stroke()
}

function drawFlag(ctx, cell, color, label) {
  const [cx, cy] = cell
  ctx.fillStyle = color
  ctx.fillRect(cx * CELL + 1, cy * CELL + 1, CELL - 2, CELL - 2)
  ctx.fillStyle = '#ffffff'
  ctx.font = `bold ${CELL * 0.6}px system-ui, sans-serif`
  ctx.textAlign = 'center'
  ctx.textBaseline = 'middle'
  ctx.fillText(label, cx * CELL + CELL / 2, cy * CELL + CELL / 2 + 1)
}

export default function GridCanvas({ grid, start, goal, plan, sim, layers, onPaint }) {
  const canvasRef = useRef(null)
  const painting = useRef(false)
  const lastCell = useRef(null)
  const rows = grid.length
  const cols = grid[0]?.length ?? 0

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    ctx.fillStyle = C.floor
    ctx.fillRect(0, 0, canvas.width, canvas.height)

    // Planner search effort: explored cells (A*) or the random tree (RRT).
    if (plan && layers.explored) {
      ctx.fillStyle = C.explored
      for (const [x, y] of plan.explored) ctx.fillRect(x * CELL, y * CELL, CELL, CELL)
      ctx.strokeStyle = C.tree
      ctx.lineWidth = 1
      ctx.beginPath()
      for (const [x1, y1, x2, y2] of plan.tree) {
        ctx.moveTo(x1 * CELL, y1 * CELL)
        ctx.lineTo(x2 * CELL, y2 * CELL)
      }
      ctx.stroke()
    }

    // Walls + faint grid lines.
    ctx.fillStyle = C.wall
    for (let y = 0; y < rows; y++)
      for (let x = 0; x < cols; x++) if (grid[y][x]) ctx.fillRect(x * CELL, y * CELL, CELL, CELL)
    ctx.strokeStyle = C.gridLine
    ctx.lineWidth = 1
    ctx.beginPath()
    for (let x = 0; x <= cols; x++) {
      ctx.moveTo(x * CELL + 0.5, 0)
      ctx.lineTo(x * CELL + 0.5, rows * CELL)
    }
    for (let y = 0; y <= rows; y++) {
      ctx.moveTo(0, y * CELL + 0.5)
      ctx.lineTo(cols * CELL, y * CELL + 0.5)
    }
    ctx.stroke()

    // Planned path.
    if (plan?.path?.length > 1) {
      ctx.strokeStyle = C.path
      ctx.lineWidth = 3.5
      ctx.lineJoin = 'round'
      ctx.lineCap = 'round'
      ctx.beginPath()
      plan.path.forEach(([x, y], i) => (i ? ctx.lineTo(x * CELL, y * CELL) : ctx.moveTo(x * CELL, y * CELL)))
      ctx.stroke()
    }

    drawFlag(ctx, start, C.start, 'S')
    drawFlag(ctx, goal, C.goal, 'G')

    if (sim) {
      // Lidar beams from the TRUE pose (that's where the real sensor is).
      if (layers.lidar) {
        const [tx, ty, tth] = sim.true_pose
        sim.scan.forEach((r, i) => {
          const a = tth + sim.beam_angles[i]
          const hx = (tx + Math.cos(a) * r) * CELL
          const hy = (ty + Math.sin(a) * r) * CELL
          ctx.strokeStyle = C.beam
          ctx.lineWidth = 1
          ctx.beginPath()
          ctx.moveTo(tx * CELL, ty * CELL)
          ctx.lineTo(hx, hy)
          ctx.stroke()
          ctx.fillStyle = C.hit
          ctx.fillRect(hx - 2, hy - 2, 4, 4)
        })
      }
      if (layers.particles) {
        ctx.fillStyle = C.particle
        for (const [x, y] of sim.particles) {
          ctx.beginPath()
          ctx.arc(x * CELL, y * CELL, 1.8, 0, Math.PI * 2)
          ctx.fill()
        }
      }
      drawRobot(ctx, sim.true_pose, C.truePose, true)
      drawRobot(ctx, sim.est_pose, C.estPose, false)
    }
  }, [grid, start, goal, plan, sim, layers, rows, cols])

  function cellFromEvent(e) {
    const rect = canvasRef.current.getBoundingClientRect()
    // The canvas may be scaled down by CSS, so convert screen px -> canvas px.
    const x = Math.floor(((e.clientX - rect.left) / rect.width) * cols)
    const y = Math.floor(((e.clientY - rect.top) / rect.height) * rows)
    if (x < 0 || y < 0 || x >= cols || y >= rows) return null
    return [x, y]
  }

  function handle(e, isStart) {
    const cell = cellFromEvent(e)
    if (!cell) return
    const key = cell.join(',')
    if (!isStart && key === lastCell.current) return
    lastCell.current = key
    onPaint(cell, isStart)
  }

  return (
    <canvas
      ref={canvasRef}
      className="grid-canvas"
      // Shrink to fit the window height on small laptop screens, keeping the aspect ratio.
      style={{ maxWidth: `calc((100vh - 190px) * ${cols / rows})` }}
      width={cols * CELL}
      height={rows * CELL}
      onPointerDown={(e) => {
        painting.current = true
        e.currentTarget.setPointerCapture(e.pointerId)
        handle(e, true)
      }}
      onPointerMove={(e) => painting.current && handle(e, false)}
      onPointerUp={() => {
        painting.current = false
        lastCell.current = null
      }}
    />
  )
}
