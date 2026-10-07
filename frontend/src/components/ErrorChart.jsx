import { useEffect, useRef, useState } from 'react'

const W = 300
const H = 110
const PAD = { l: 34, r: 8, t: 8, b: 22 }

// Line chart of localization error (|estimate - truth| in cells) per step,
// with a hover crosshair that shows the exact value.
export default function ErrorChart({ errors }) {
  const ref = useRef(null)
  const [hover, setHover] = useState(null)

  const n = errors.length
  const maxY = Math.max(0.5, ...errors) * 1.1
  const sx = (i) => PAD.l + (i / Math.max(1, n - 1)) * (W - PAD.l - PAD.r)
  const sy = (v) => H - PAD.b - (v / maxY) * (H - PAD.t - PAD.b)

  useEffect(() => {
    const canvas = ref.current
    const dpr = window.devicePixelRatio || 1
    canvas.width = W * dpr
    canvas.height = H * dpr
    const ctx = canvas.getContext('2d')
    ctx.scale(dpr, dpr)
    const css = getComputedStyle(canvas)
    const ink = css.getPropertyValue('--muted').trim()
    const grid = css.getPropertyValue('--grid').trim()
    const line = css.getPropertyValue('--series').trim()

    ctx.clearRect(0, 0, W, H)
    ctx.font = '11px system-ui, sans-serif'
    ctx.fillStyle = ink
    ctx.strokeStyle = grid
    ctx.lineWidth = 1
    ctx.textAlign = 'right'
    ctx.textBaseline = 'middle'
    for (let k = 0; k <= 3; k++) {
      const v = (maxY * k) / 3
      const y = Math.round(sy(v)) + 0.5
      ctx.beginPath()
      ctx.moveTo(PAD.l, y)
      ctx.lineTo(W - PAD.r, y)
      ctx.stroke()
      ctx.fillText(v.toFixed(1), PAD.l - 6, y)
    }
    ctx.textAlign = 'left'
    ctx.textBaseline = 'top'
    ctx.fillText('step 0', PAD.l, H - PAD.b + 6)
    ctx.textAlign = 'right'
    ctx.fillText(`step ${Math.max(0, n - 1)}`, W - PAD.r, H - PAD.b + 6)

    if (n > 1) {
      ctx.strokeStyle = line
      ctx.lineWidth = 2
      ctx.lineJoin = 'round'
      ctx.beginPath()
      errors.forEach((v, i) => (i ? ctx.lineTo(sx(i), sy(v)) : ctx.moveTo(sx(i), sy(v))))
      ctx.stroke()
    }
    if (hover !== null && hover < n) {
      const x = sx(hover)
      ctx.strokeStyle = ink
      ctx.lineWidth = 1
      ctx.beginPath()
      ctx.moveTo(x, PAD.t)
      ctx.lineTo(x, H - PAD.b)
      ctx.stroke()
      ctx.fillStyle = line
      ctx.beginPath()
      ctx.arc(x, sy(errors[hover]), 4, 0, Math.PI * 2)
      ctx.fill()
    }
  })

  function onMove(e) {
    if (n < 2) return
    const rect = ref.current.getBoundingClientRect()
    const px = ((e.clientX - rect.left) / rect.width) * W
    const i = Math.round(((px - PAD.l) / (W - PAD.l - PAD.r)) * (n - 1))
    setHover(Math.min(n - 1, Math.max(0, i)))
  }

  return (
    <div className="chart-wrap">
      <canvas
        ref={ref}
        className="error-chart"
        style={{ width: W, height: H }}
        onPointerMove={onMove}
        onPointerLeave={() => setHover(null)}
        aria-label="Localization error over time"
      />
      {hover !== null && hover < n && (
        <div className="chart-tip" style={{ left: `${(sx(hover) / W) * 100}%` }}>
          step {hover}: <b>{errors[hover].toFixed(3)}</b> cells
        </div>
      )}
    </div>
  )
}
