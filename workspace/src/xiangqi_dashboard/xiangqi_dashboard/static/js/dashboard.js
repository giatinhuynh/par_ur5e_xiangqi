/**
 * Xiangqi Dashboard JS
 * Connects to the Flask-SocketIO server and renders the board + state.
 */

'use strict';

const socket = io();
const canvas = document.getElementById('board-canvas');
const ctx = canvas.getContext('2d');

// Board dimensions (must match CSS canvas size)
const W = 450, H = 500;
const MARGIN = 30;
const CELL = (W - 2 * MARGIN) / 8;   // 9 files = 8 intervals
const FILES = 9, RANKS = 10;

// Piece codes -> display strings
const PIECE_NAMES = {
  1: ['将','帅'], 2: ['士','仕'], 3: ['象','相'],
  4: ['马','马'], 5: ['车','车'], 6: ['炮','炮'], 7: ['卒','兵']
};

function pieceLabel(code) {
  if (code === 0) return '';
  const abs = Math.abs(code);
  const isRed = code > 0;
  const pair = PIECE_NAMES[abs];
  if (!pair) return '?';
  return pair[isRed ? 1 : 0];
}

function drawBoard(grid) {
  ctx.clearRect(0, 0, W, H);

  // Background
  ctx.fillStyle = '#2a1f0a';
  ctx.fillRect(0, 0, W, H);

  // Grid lines
  ctx.strokeStyle = '#8b6914';
  ctx.lineWidth = 1;

  for (let f = 0; f < FILES; f++) {
    const x = MARGIN + f * CELL;
    // Top half (ranks 5-9)
    ctx.beginPath();
    ctx.moveTo(x, MARGIN);
    ctx.lineTo(x, MARGIN + 4 * CELL);
    ctx.stroke();
    // Bottom half (ranks 0-4)
    ctx.beginPath();
    ctx.moveTo(x, MARGIN + 5 * CELL);
    ctx.lineTo(x, MARGIN + 9 * CELL);
    ctx.stroke();
  }

  for (let r = 0; r < RANKS; r++) {
    const y = MARGIN + r * CELL;
    ctx.beginPath();
    ctx.moveTo(MARGIN, y);
    ctx.lineTo(MARGIN + 8 * CELL, y);
    ctx.stroke();
  }

  // River label
  ctx.fillStyle = '#8b6914';
  ctx.font = '11px serif';
  ctx.textAlign = 'center';
  ctx.fillText('楚 河', MARGIN + 2 * CELL, MARGIN + 4.5 * CELL + 4);
  ctx.fillText('汉 界', MARGIN + 6 * CELL, MARGIN + 4.5 * CELL + 4);

  // Palace diagonals
  ctx.strokeStyle = '#8b6914';
  ctx.lineWidth = 0.8;
  // Red palace (ranks 0-2, files 3-5)
  ctx.beginPath();
  ctx.moveTo(MARGIN + 3 * CELL, MARGIN + 9 * CELL);
  ctx.lineTo(MARGIN + 5 * CELL, MARGIN + 7 * CELL);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(MARGIN + 5 * CELL, MARGIN + 9 * CELL);
  ctx.lineTo(MARGIN + 3 * CELL, MARGIN + 7 * CELL);
  ctx.stroke();
  // Black palace (ranks 7-9)
  ctx.beginPath();
  ctx.moveTo(MARGIN + 3 * CELL, MARGIN + 0 * CELL);
  ctx.lineTo(MARGIN + 5 * CELL, MARGIN + 2 * CELL);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(MARGIN + 5 * CELL, MARGIN + 0 * CELL);
  ctx.lineTo(MARGIN + 3 * CELL, MARGIN + 2 * CELL);
  ctx.stroke();

  // Pieces
  for (let rank = 0; rank < RANKS; rank++) {
    for (let file = 0; file < FILES; file++) {
      const idx = rank * FILES + file;
      const code = grid[idx] || 0;
      if (code === 0) continue;

      // Note: rank 9 = top of canvas (black side), rank 0 = bottom (red)
      const cx = MARGIN + file * CELL;
      const cy = MARGIN + (9 - rank) * CELL;
      const r = CELL * 0.42;

      // Piece circle
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.fillStyle = code > 0 ? '#c0392b' : '#111';
      ctx.fill();
      ctx.strokeStyle = code > 0 ? '#e74c3c' : '#555';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Piece label
      ctx.fillStyle = code > 0 ? '#fff' : '#ccc';
      ctx.font = `bold ${Math.round(CELL * 0.45)}px serif`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(pieceLabel(code), cx, cy);
    }
  }
}

function updateEvalBar(evalCp) {
  const bar = document.getElementById('eval-bar');
  // Map -5000..+5000 cp to 0..100% width
  const clamped = Math.max(-5000, Math.min(5000, evalCp));
  const pct = ((clamped + 5000) / 10000) * 100;
  bar.style.width = pct + '%';
  bar.style.background = evalCp >= 0 ? '#c0392b' : '#444';
}

function addMoveEntry(entry) {
  const list = document.getElementById('move-history-list');
  const div = document.createElement('div');
  div.className = 'move-entry ' + (entry.is_red ? 'red-move' : 'black-move');
  const side = entry.is_red ? '🔴' : '⚫';
  div.innerHTML = `
    <span class="move-notation">${entry.move}</span>
    <span>${side} ${entry.engine}</span>
    <span class="move-meta">d${entry.depth} ${entry.eval > 0 ? '+' : ''}${entry.eval}cp ${entry.time}s</span>
  `;
  list.prepend(div);
}

socket.on('state_update', (state) => {
  // Board
  if (state.board_grid) drawBoard(state.board_grid);

  // Turn indicator
  const turnEl = document.getElementById('turn-indicator');
  if (state.game_result && state.game_result !== 'ongoing') {
    const reason = state.game_result_reason || '';
    let label = '';
    if (state.game_result === 'red_wins') label = 'Red wins';
    else if (state.game_result === 'black_wins') label = 'Black wins';
    else if (state.game_result === 'draw') label = 'Draw';
    else label = 'Game over';
    turnEl.textContent = reason ? `${label} (${reason})` : label;
    turnEl.className = 'turn-indicator turn-over';
  } else {
    turnEl.textContent = state.is_red_turn ? 'Red to move' : 'Black to move';
    turnEl.className = 'turn-indicator ' + (state.is_red_turn ? 'turn-red' : 'turn-black');
  }

  // Confidence
  document.getElementById('confidence').textContent =
    `Detection confidence: ${(state.detection_confidence * 100).toFixed(1)}%`;

  // System header
  const sysEl = document.getElementById('system-status');
  const estop = state.estop_active;
  if (estop) {
    sysEl.textContent = 'E-STOP ACTIVE';
  } else if (state.game_result && state.game_result !== 'ongoing') {
    const reason = state.game_result_reason || '';
    let label = '';
    if (state.game_result === 'red_wins') label = 'Red wins';
    else if (state.game_result === 'black_wins') label = 'Black wins';
    else if (state.game_result === 'draw') label = 'Draw';
    else label = 'Game over';
    sysEl.textContent = reason ? `${label} (${reason})` : label;
  } else {
    sysEl.textContent = `System: ${state.system_state || 'OK'}`;
  }
  sysEl.className = estop ? 'status-err' : 'status-ok';

  // AI panel
  document.getElementById('ai-engine-label').textContent = `Engine: ${state.engine_type}`;
  document.getElementById('eval-score').textContent =
    `Evaluation: ${state.evaluation_cp > 0 ? '+' : ''}${state.evaluation_cp} cp`;
  document.getElementById('search-info').textContent =
    `Depth: ${state.depth_reached} | Time: ${state.thinking_time}s`;
  document.getElementById('best-move').textContent = `Best move: ${state.best_move || '--'}`;
  document.getElementById('ponder-move').textContent = `Ponder: ${state.ponder_move || '--'}`;
  updateEvalBar(state.evaluation_cp);

  // Status table
  document.getElementById('st-game-state').textContent = state.game_status || '--';
  if (state.game_result && state.game_result !== 'ongoing') {
    const reason = state.game_result_reason || '';
    let label = '';
    if (state.game_result === 'red_wins') label = 'Red wins';
    else if (state.game_result === 'black_wins') label = 'Black wins';
    else if (state.game_result === 'draw') label = 'Draw';
    else label = 'Game over';
    document.getElementById('st-game-result').textContent =
      reason ? `${label} (${reason})` : label;
  } else {
    document.getElementById('st-game-result').textContent = '--';
  }
  document.getElementById('st-move-count').textContent = state.move_count ?? '--';
  document.getElementById('st-gripper').textContent = state.gripper_active ? 'Gripping (RG2 closed)' : 'Idle';
  const estopCell = document.getElementById('st-estop');
  estopCell.textContent = estop ? 'ACTIVE' : 'OK';
  estopCell.style.color = estop ? '#e74c3c' : '#27ae60';

  // Move history (only new entries)
  const list = document.getElementById('move-history-list');
  const currentCount = list.children.length;
  const histLen = (state.move_history || []).length;
  if (histLen > currentCount) {
    const newEntries = state.move_history.slice(currentCount);
    newEntries.forEach(addMoveEntry);
  }
});

// Buttons
document.getElementById('btn-new-game').addEventListener('click', () => {
  if (confirm('Start a new game?')) fetch('/api/new_game', { method: 'POST' });
});

document.getElementById('btn-human-ready').addEventListener('click', () => {
  fetch('/api/human_ready', { method: 'POST' });
});

document.getElementById('btn-resync').addEventListener('click', () => {
  if (confirm('Resync UI/game state from vision? This overwrites the game manager FEN.')) {
    fetch('/api/resync_from_vision', { method: 'POST' });
  }
});

let estopActive = false;
document.getElementById('btn-estop').addEventListener('click', () => {
  estopActive = !estopActive;
  fetch('/api/emergency_stop', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ active: estopActive }),
  });
  document.getElementById('btn-estop').textContent =
    estopActive ? 'Release E-Stop' : 'Emergency Stop';
});

const diffSlider = document.getElementById('difficulty-slider');
const diffValue = document.getElementById('difficulty-value');
diffSlider.addEventListener('input', () => { diffValue.textContent = diffSlider.value; });

document.getElementById('btn-set-engine').addEventListener('click', () => {
  fetch('/api/set_engine', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      engine_type: document.getElementById('engine-select').value,
      difficulty: parseInt(diffSlider.value),
    }),
  });
});

// Initial board render
drawBoard(new Array(90).fill(0));
