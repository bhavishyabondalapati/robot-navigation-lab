// Small helpers for talking to the FastAPI backend.

export async function fetchMap(name) {
  const res = await fetch(`/api/maps/${name}`)
  if (!res.ok) throw new Error(`could not load map ${name}`)
  return res.json()
}

export async function planOnce({ grid, start, goal, planner, seed }) {
  const res = await fetch('/api/plan', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ grid, start, goal, planner, seed }),
  })
  if (!res.ok) throw new Error(`planning failed (${res.status})`)
  return res.json()
}

// Opens the live-simulation socket. `onMessage` receives parsed JSON messages.
export function openSimSocket(onMessage, onClose) {
  const proto = window.location.protocol === 'https:' ? 'wss' : 'ws'
  const ws = new WebSocket(`${proto}://${window.location.host}/ws/simulate`)
  ws.onmessage = (e) => onMessage(JSON.parse(e.data))
  ws.onclose = onClose
  return ws
}
