// Side-by-side numbers for the most recent A* and RRT plans.
const ROWS = [
  ['Found path', (r) => (r.success ? 'yes' : 'no')],
  ['Path length', (r) => (r.success ? r.path_length.toFixed(1) : '–')],
  ['Nodes expanded', (r) => r.nodes_expanded],
  ['Planning time', (r) => `${r.planning_time_ms.toFixed(1)} ms`],
]

export default function StatsTable({ results }) {
  const cols = [
    ['astar', 'A*'],
    ['rrt', 'RRT'],
  ]
  if (!results.astar && !results.rrt) {
    return <p className="hint">Press Plan or Compare to see numbers.</p>
  }
  return (
    <table className="stats">
      <thead>
        <tr>
          <th />
          {cols.map(([k, label]) => <th key={k}>{label}</th>)}
        </tr>
      </thead>
      <tbody>
        {ROWS.map(([label, fmt]) => (
          <tr key={label}>
            <td>{label}</td>
            {cols.map(([k]) => (
              <td key={k} className="mono">{results[k] ? fmt(results[k]) : '–'}</td>
            ))}
          </tr>
        ))}
      </tbody>
    </table>
  )
}
