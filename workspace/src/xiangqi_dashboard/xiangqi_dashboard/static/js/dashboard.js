/**
 * Xiangqi Dashboard — dashboard.js
 * Interactive board, mode switching, real-time state via SocketIO.
 */

'use strict';

// ── Constants ─────────────────────────────────────────────────────

const COLS = 9, ROWS = 10;
const W = 480, H = 533;
const MARGIN_X = 30, MARGIN_Y = 27;
const CELL_W = (W - 2 * MARGIN_X) / (COLS - 1);   // 52.5px
const CELL_H = (H - 2 * MARGIN_Y) / (ROWS - 1);   // 53.22px

// Piece codes → {red label, black label}
const PIECE_LABELS = {
  1: { r: '帅', b: '将' },
  2: { r: '仕', b: '士' },
  3: { r: '相', b: '象' },
  4: { r: '马', b: '马' },
  5: { r: '车', b: '车' },
  6: { r: '炮', b: '炮' },
  7: { r: '兵', b: '卒' },
};

// Piece size
const PIECE_R = 20;

// Move-coordinate helpers: board uses rank 0=bottom(Red home) to 9=top(Black home)
// pyffish / UCI uses: file a-i (0-8), rank 1-10 (1=Red home, 10=Black home)
function gridIdxToCoord(idx) {
  const rank = Math.floor(idx / COLS);   // 0 (Red home) … 9 (Black home)
  const file = idx % COLS;
  return { file, rank };
}
function coordToGridIdx(file, rank) { return rank * COLS + file; }

// UCI: file char a-i, rank 1-10 (1=Red home, 10=Black home)
// board rank 0 = pyffish rank 1 (Red home), board rank 9 = pyffish rank 10
function coordToUCI(file, rank) {
  return String.fromCharCode(97 + file) + (rank + 1).toString();
}
function uciToCoord(uci) {
  if (!uci || uci.length < 4) return null;
  const file = uci.charCodeAt(0) - 97;
  const rank = parseInt(uci[1]) - 1 + (uci.length >= 4 && uci[1] === '1' && uci[2] === '0' ? 9 : 0);
  const toFile = uci.charCodeAt(uci.length === 5 ? 2 : 2) - 97;
  const toRank = parseInt(uci.slice(uci.length === 5 ? 3 : 3)) - 1;
  return { fromFile: file, fromRank: rank, toFile, toRank };
}

// ── State ─────────────────────────────────────────────────────────

let state = {
  board_grid: Array(90).fill(0),
  fen: '',
  game_status: 'idle',
  is_red_turn: true,
  move_count: 0,
  engine_type: '--',
  red_engine: 'minimax',
  black_engine: 'minimax',
  stockfish_difficulty: 20,
  evaluation_cp: 0,
  depth_reached: 0,
  thinking_time: 0,
  best_move: '',
  ponder_move: '',
  move_history: [],
  gripper_active: false,
  estop_active: false,
  system_state: 'starting',
  game_result: 'ongoing',
  game_result_reason: '',
  simulation_mode: true,
  game_mode: 'ai_vs_human',
  last_alert: '',
};

// Human color choice in AI vs Human mode ('red' | 'black')
let humanColor = 'red';

function setHumanColor(color) {
  humanColor = color;
  document.getElementById('btn-play-red').classList.toggle('active', color === 'red');
  document.getElementById('btn-play-black').classList.toggle('active', color === 'black');
  updateEngineSelectors();
  fetch('/api/set_human_color', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ color }),
  }).catch(() => {});
}

// Interactive selection state
let selectedIdx = null;
let legalDests  = [];
let legalDestsRequestId = 0;
let lastMove    = null;

// True from Start/Reset click until backend leaves idle (or timeout)
let gameStarting = false;
let gameStartingTimer = null;
const GAME_START_TIMEOUT_MS = 45000;

// ── Flow helpers ──────────────────────────────────────────────────

function isGameStarting() {
  return !!gameStarting;
}

function beginGameStart(label) {
  gameStarting = true;
  if (gameStartingTimer) clearTimeout(gameStartingTimer);
  const text = document.getElementById('board-loading-text');
  if (text) text.textContent = label || 'Starting game…';
  gameStartingTimer = setTimeout(() => {
    if (!gameStarting) return;
    gameStarting = false;
    showToast('Start timed out — try Reset or check logs');
    renderAll();
  }, GAME_START_TIMEOUT_MS);
  renderAll();
}

function endGameStart() {
  if (!gameStarting) return;
  gameStarting = false;
  if (gameStartingTimer) {
    clearTimeout(gameStartingTimer);
    gameStartingTimer = null;
  }
}

function checkGameStartComplete() {
  if (!gameStarting) return;
  const gs = (state.game_status || 'idle').toLowerCase();
  if (gs !== 'idle' && gs !== 'game_over') {
    endGameStart();
    return;
  }
  if ((state.move_count || 0) > 0) {
    endGameStart();
  }
}

function isGameInProgress() {
  const gs = (state.game_status || 'idle').toLowerCase();
  return gs !== 'idle' && gs !== 'game_over';
}

function canChangeMode() {
  return !isGameInProgress() && !isGameStarting() && !state.estop_active;
}

function getGamePhase() {
  if (state.estop_active) return 'estop';
  if (gameStarting) return 'starting';
  const gs = (state.game_status || 'idle').toLowerCase();
  if (gs === 'game_over') return 'over';
  if (gs === 'idle') return 'setup';
  return 'playing';
}

function formatModeLabel(mode) {
  if (mode === 'ai_vs_ai') return 'AI vs AI';
  if (mode === 'ai_vs_human') return 'AI vs Human';
  return mode || '--';
}

function formatActivity(status) {
  const gs = (status || '').toLowerCase();
  const aiVsAi = state.game_mode === 'ai_vs_ai';
  if (aiVsAi && gs === 'computing_ai') {
    const side = state.is_red_turn ? 'Red AI' : 'Black AI';
    return `${side} thinking`;
  }
  if (aiVsAi && gs === 'waiting_human') {
    return 'Starting AI vs AI…';
  }
  if (aiVsAi && gs === 'executing_move') {
    return 'AI move in progress';
  }
  if (gs === 'waiting_human') {
    const side = humanColor === 'red' ? 'Red' : 'Black';
    return `Waiting for you (${side})`;
  }
  const map = {
    idle: 'Idle — choose mode, then Start',
    detecting_move: 'Detecting move',
    validating_move: 'Validating move',
    computing_ai: 'AI thinking',
    executing_move: 'Robot moving',
    game_over: 'Game over',
  };
  return map[gs] || (status || '--').replace(/_/g, ' ');
}

function clampEvalCp(cp) {
  const n = Number(cp) || 0;
  if (Math.abs(n) > 50000) return null;
  return Math.max(-1500, Math.min(1500, n));
}

function formatGameResult(result, reason) {
  const r = (result || 'ongoing').toLowerCase();
  const titles = {
    ongoing: 'Game in progress',
    red_wins: 'Red wins',
    black_wins: 'Black wins',
    draw: 'Draw',
    unknown: 'Game ended',
  };
  const reasonLabels = {
    checkmate: 'Checkmate',
    stalemate: 'Stalemate',
    win_by_rule_or_resign: 'Win by rule',
    draw_by_rule: 'Draw by rule',
    no_legal_moves: 'No legal moves',
    threefold_repetition: 'Threefold repetition',
    draw_by_repetition: 'Draw by repetition',
    perpetual_rule: 'Perpetual check / chasing',
    insufficient_material: 'Insufficient material',
    move_limit: 'Move limit reached (150 full moves)',
    ai_engine_failed: 'AI engine error',
  };
  const title = titles[r] || r.replace(/_/g, ' ');
  if (r === 'ongoing') {
    return { title: '—', detail: 'Game not finished yet', css: 'result-ongoing' };
  }
  const detail = reasonLabels[(reason || '').toLowerCase()] || (reason || '').replace(/_/g, ' ');
  let css = '';
  if (r === 'red_wins') css = 'result-win-red';
  else if (r === 'black_wins') css = 'result-win-black';
  else if (r === 'draw') css = 'result-draw';
  return { title, detail: detail || 'Game over', css };
}

// ── Canvas ────────────────────────────────────────────────────────

const canvas = document.getElementById('board-canvas');
const ctx    = canvas.getContext('2d');

// Convert board pixel → grid index
function pixelToGridIdx(px, py) {
  const file = Math.round((px - MARGIN_X) / CELL_W);
  const rank = Math.round((py - MARGIN_Y) / CELL_H);
  if (file < 0 || file >= COLS || rank < 0 || rank >= ROWS) return null;
  // In the canvas rank 0 = top (Black home, rank 10 in UCI)
  // We draw rank 9 (Red home) at the bottom => canvas row 0 = board rank 9
  // Actually: we draw board rank 9 at canvas top, rank 0 at canvas bottom
  // canvas_row_from_top = 9 - board_rank
  const boardRank = 9 - rank;
  return coordToGridIdx(file, boardRank);
}

// Board grid index → canvas pixel centre
function gridIdxToPixel(idx) {
  const { file, rank } = gridIdxToCoord(idx);
  const canvasRow = 9 - rank;   // rank 9 = top, rank 0 = bottom
  return {
    x: MARGIN_X + file * CELL_W,
    y: MARGIN_Y + canvasRow * CELL_H,
  };
}

// ── Drawing ───────────────────────────────────────────────────────

function drawBoard() {
  ctx.clearRect(0, 0, W, H);

  // Background wood
  const woodGrad = ctx.createLinearGradient(0, 0, W, H);
  woodGrad.addColorStop(0,   '#c8822a');
  woodGrad.addColorStop(0.5, '#d4923a');
  woodGrad.addColorStop(1,   '#b87220');
  ctx.fillStyle = woodGrad;
  ctx.fillRect(0, 0, W, H);

  // Grid lines
  ctx.strokeStyle = 'rgba(80,40,5,.7)';
  ctx.lineWidth   = 1;

  // Vertical lines (files) — break at river
  for (let f = 0; f < COLS; f++) {
    const x = MARGIN_X + f * CELL_W;
    // Top half (canvas rows 0-4 = board ranks 9-5)
    ctx.beginPath();
    ctx.moveTo(x, MARGIN_Y);
    ctx.lineTo(x, MARGIN_Y + 4 * CELL_H);
    ctx.stroke();
    // Bottom half (canvas rows 5-9 = board ranks 4-0)
    ctx.beginPath();
    ctx.moveTo(x, MARGIN_Y + 5 * CELL_H);
    ctx.lineTo(x, MARGIN_Y + 9 * CELL_H);
    ctx.stroke();
  }

  // Horizontal lines (ranks)
  for (let r = 0; r < ROWS; r++) {
    const y = MARGIN_Y + r * CELL_H;
    ctx.beginPath();
    ctx.moveTo(MARGIN_X, y);
    ctx.lineTo(MARGIN_X + 8 * CELL_W, y);
    ctx.stroke();
  }

  // River
  const riverY = MARGIN_Y + 4.5 * CELL_H;
  ctx.fillStyle = 'rgba(0,0,0,.12)';
  ctx.fillRect(MARGIN_X + 1, MARGIN_Y + 4 * CELL_H + 1, 8 * CELL_W - 2, CELL_H - 2);

  ctx.font         = 'bold 18px "Noto Serif SC", serif';
  ctx.fillStyle    = 'rgba(60,25,0,.55)';
  ctx.textAlign    = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('楚 河', MARGIN_X + 2 * CELL_W, riverY);
  ctx.fillText('汉 界', MARGIN_X + 6 * CELL_W, riverY);

  // Palace diagonals
  ctx.strokeStyle = 'rgba(80,40,5,.6)';
  ctx.lineWidth   = .8;
  // Red palace (board ranks 0-2, files 3-5) → canvas rows 9-7
  const rp = {
    tl: { x: MARGIN_X + 3 * CELL_W, y: MARGIN_Y + 7 * CELL_H },
    tr: { x: MARGIN_X + 5 * CELL_W, y: MARGIN_Y + 7 * CELL_H },
    bl: { x: MARGIN_X + 3 * CELL_W, y: MARGIN_Y + 9 * CELL_H },
    br: { x: MARGIN_X + 5 * CELL_W, y: MARGIN_Y + 9 * CELL_H },
  };
  ctx.beginPath(); ctx.moveTo(rp.tl.x, rp.tl.y); ctx.lineTo(rp.br.x, rp.br.y); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(rp.tr.x, rp.tr.y); ctx.lineTo(rp.bl.x, rp.bl.y); ctx.stroke();
  // Black palace (board ranks 9-7, files 3-5) → canvas rows 0-2
  const bp = {
    tl: { x: MARGIN_X + 3 * CELL_W, y: MARGIN_Y + 0 * CELL_H },
    tr: { x: MARGIN_X + 5 * CELL_W, y: MARGIN_Y + 0 * CELL_H },
    bl: { x: MARGIN_X + 3 * CELL_W, y: MARGIN_Y + 2 * CELL_H },
    br: { x: MARGIN_X + 5 * CELL_W, y: MARGIN_Y + 2 * CELL_H },
  };
  ctx.beginPath(); ctx.moveTo(bp.tl.x, bp.tl.y); ctx.lineTo(bp.br.x, bp.br.y); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(bp.tr.x, bp.tr.y); ctx.lineTo(bp.bl.x, bp.bl.y); ctx.stroke();

  // Highlight last-move squares
  if (lastMove) {
    for (const idx of [lastMove.from, lastMove.to]) {
      if (idx == null) continue;
      const { x, y } = gridIdxToPixel(idx);
      ctx.fillStyle = 'rgba(255,200,0,.22)';
      ctx.beginPath();
      ctx.arc(x, y, PIECE_R + 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // Highlight legal destination squares
  for (const idx of legalDests) {
    const { x, y } = gridIdxToPixel(idx);
    const piece = state.board_grid[idx];
    if (piece !== 0) {
      // Capture highlight (red ring)
      ctx.strokeStyle = 'rgba(231,76,60,.85)';
      ctx.lineWidth   = 2.5;
      ctx.beginPath();
      ctx.arc(x, y, PIECE_R + 3, 0, Math.PI * 2);
      ctx.stroke();
    } else {
      // Empty square dot
      ctx.fillStyle = 'rgba(50,200,100,.6)';
      ctx.beginPath();
      ctx.arc(x, y, 7, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // Draw pieces
  const grid = state.board_grid;
  for (let i = 0; i < 90; i++) {
    const code = grid[i];
    if (code === 0) continue;

    const isRed = code > 0;
    const { x, y } = gridIdxToPixel(i);
    const isSelected = (i === selectedIdx);
    const abs = Math.abs(code);
    const label = PIECE_LABELS[abs] ? (isRed ? PIECE_LABELS[abs].r : PIECE_LABELS[abs].b) : '?';

    drawPiece(ctx, x, y, label, isRed, isSelected);
  }

  // Selection highlight
  if (selectedIdx !== null) {
    const { x, y } = gridIdxToPixel(selectedIdx);
    ctx.strokeStyle = 'rgba(255,220,50,0.9)';
    ctx.lineWidth   = 3;
    ctx.beginPath();
    ctx.arc(x, y, PIECE_R + 5, 0, Math.PI * 2);
    ctx.stroke();
  }
}

function drawPiece(ctx, x, y, label, isRed, isSelected) {
  // Outer ring (shadow)
  ctx.beginPath();
  ctx.arc(x, y, PIECE_R + 2, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(0,0,0,.35)';
  ctx.fill();

  // Piece circle body
  const grad = ctx.createRadialGradient(x - 4, y - 4, 2, x, y, PIECE_R);
  if (isRed) {
    grad.addColorStop(0, isSelected ? '#ff9077' : '#e85a4b');
    grad.addColorStop(1, isSelected ? '#aa2010' : '#8b1a10');
  } else {
    grad.addColorStop(0, isSelected ? '#555' : '#333');
    grad.addColorStop(1, '#111');
  }
  ctx.beginPath();
  ctx.arc(x, y, PIECE_R, 0, Math.PI * 2);
  ctx.fillStyle = grad;
  ctx.fill();

  // Outer border ring
  ctx.strokeStyle = isRed ? 'rgba(255,180,140,.5)' : 'rgba(180,180,180,.3)';
  ctx.lineWidth   = 1.5;
  ctx.beginPath();
  ctx.arc(x, y, PIECE_R, 0, Math.PI * 2);
  ctx.stroke();

  // Inner ring (decorative)
  ctx.strokeStyle = isRed ? 'rgba(255,200,160,.4)' : 'rgba(160,160,160,.3)';
  ctx.lineWidth   = 1;
  ctx.beginPath();
  ctx.arc(x, y, PIECE_R - 4, 0, Math.PI * 2);
  ctx.stroke();

  // Chinese character
  ctx.font         = `bold 16px "Noto Serif SC", "Microsoft YaHei", serif`;
  ctx.fillStyle    = isRed ? '#ffd0c0' : '#c8c8c8';
  ctx.textAlign    = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, x, y + 1);
}

// ── Socket.IO ─────────────────────────────────────────────────────

const socket = io({
  transports: ['polling'],
  reconnection: true,
  reconnectionDelay: 1000,
  reconnectionDelayMax: 5000,
  timeout: 20000,
});

/** Pull full state from REST (used on connect/reconnect and when socket drops). */
function fetchStateSnapshot() {
  return fetch('/api/state')
    .then((r) => r.json())
    .then((data) => {
      state = { ...state, ...data };
      checkGameStartComplete();
      renderAll();
      return data;
    });
}

socket.on('connect', () => {
  document.getElementById('connection-dot').className = 'conn-dot connected';
  document.getElementById('system-status-label').textContent = 'Connected';
  // Resync after reconnect — backend may have advanced many moves while UI was stale
  fetchStateSnapshot().catch(() => {});
  updateSyncPolling();
});

socket.on('disconnect', () => {
  document.getElementById('connection-dot').className = 'conn-dot disconnected';
  document.getElementById('system-status-label').textContent = 'Disconnected';
  updateSyncPolling();
});

socket.on('state_update', (data) => {
  const prevAlert = state.last_alert;
  state = { ...state, ...data };
  checkGameStartComplete();
  renderAll();

  // Show alert toast if new
  if (state.last_alert && state.last_alert !== prevAlert) {
    showToast(state.last_alert);
  }
});

// ── Rendering ─────────────────────────────────────────────────────

function renderAll() {
  estopActive = !!state.estop_active;
  checkGameStartComplete();
  const humanIsRed = humanColor === 'red';
  const humanTurn =
    state.game_mode === 'ai_vs_human' &&
    (state.game_status || '').toLowerCase() === 'waiting_human' &&
    (humanIsRed ? state.is_red_turn : !state.is_red_turn);
  if (!humanTurn) {
    selectedIdx = null;
    legalDests = [];
    legalDestsRequestId += 1;
  }
  drawBoard();
  updateHeader();
  updateBoardLoading();
  updateModeBar();
  updateFlowBanner();
  updateGameResultBanner();
  updateAIPanel();
  updateSystemPanel();
  updateHistory();
  updateSyncPolling();
}

function updateBoardLoading() {
  const overlay = document.getElementById('board-loading');
  if (!overlay) return;
  const starting = isGameStarting();
  overlay.classList.toggle('hidden', !starting);
  const wrap = document.getElementById('board-wrapper');
  if (wrap) wrap.classList.toggle('is-loading', starting);
}

function updateHeader() {
  const simBadge = document.getElementById('sim-badge');
  if (state.simulation_mode) {
    simBadge.classList.remove('hidden');
  } else {
    simBadge.classList.add('hidden');
  }

  const sysLabel = document.getElementById('system-status-label');
  if (!sysLabel) return;
  if (isGameStarting()) {
    sysLabel.textContent = 'Starting game…';
  } else if (socket.connected) {
    sysLabel.textContent = state.system_state || 'Connected';
  }
}

function updateModeBar() {
  const sim = !!state.simulation_mode;
  const modeBarSim = document.getElementById('mode-bar-sim');
  const hwBanner = document.getElementById('hardware-play-banner');
  const lockHint = document.getElementById('mode-lock-hint');
  const btnAiAi = document.getElementById('btn-ai-vs-ai');
  const btnHuman = document.getElementById('btn-ai-vs-human');
  const newGameBtn = document.getElementById('btn-new-game');
  const stopBtn    = document.getElementById('btn-stop-game');
  const resetBtn   = document.getElementById('btn-reset-game');
  const inProgress = isGameInProgress();
  const starting = isGameStarting();
  const busy = starting || inProgress;

  if (modeBarSim) modeBarSim.classList.toggle('hidden', !sim);
  if (hwBanner) hwBanner.classList.toggle('hidden', sim);

  if (!sim && state.game_mode === 'ai_vs_ai') {
    state.game_mode = 'ai_vs_human';
  }

  const modeLocked = sim && (!canChangeMode() || starting);
  if (lockHint) {
    lockHint.classList.toggle('hidden', !modeLocked);
    lockHint.textContent = starting ? 'Starting…' : 'Locked while playing';
  }
  if (btnAiAi) {
    btnAiAi.classList.toggle('active', state.game_mode === 'ai_vs_ai');
    btnAiAi.disabled = modeLocked;
  }
  if (btnHuman) {
    btnHuman.classList.toggle('active', state.game_mode === 'ai_vs_human');
    btnHuman.disabled = modeLocked;
  }
  if (newGameBtn) {
    newGameBtn.disabled = !!state.estop_active || busy;
    newGameBtn.classList.toggle('is-loading', starting);
    newGameBtn.textContent = starting ? 'Starting…' : 'Start';
  }
  if (stopBtn) stopBtn.disabled = !busy || !!state.estop_active;
  if (resetBtn) resetBtn.disabled = !!state.estop_active;

  updateEngineSelectors();

  const canvasEl = document.getElementById('board-canvas');
  const isHumanMode = sim && (state.game_mode === 'ai_vs_human');
  const isRedHuman = state.is_red_turn;
  const humanTurn = isHumanMode && isRedHuman
    && (state.game_status || '').toLowerCase() === 'waiting_human'
    && !state.estop_active;

  if (canvasEl) canvasEl.classList.toggle('selectable', humanTurn && !starting);

  // Turn indicator
  const turnDot   = document.getElementById('turn-dot');
  const turnLabel = document.getElementById('turn-label');
  turnDot.className = 'turn-dot ' + (state.is_red_turn ? 'red' : 'black');
  const gs = (state.game_status || '').toLowerCase();
  const phase = getGamePhase();
  if (phase === 'starting') {
    turnLabel.textContent = 'Starting game — please wait…';
  } else if (phase === 'setup') {
    turnLabel.textContent = 'Choose mode, then Start Game';
  } else if (phase === 'estop') {
    turnLabel.textContent = 'E-STOP — game halted';
  } else if (phase === 'over') {
    const { title } = formatGameResult(state.game_result, state.game_result_reason);
    turnLabel.textContent = title;
  } else if (gs === 'computing_ai') {
    turnLabel.textContent =
      state.game_mode === 'ai_vs_ai' ? 'AI vs AI — thinking…' : 'AI thinking…';
  } else if (gs === 'game_over') {
    turnLabel.textContent = 'Game over';
  } else if (humanTurn && !starting) {
    const humanSide = humanColor === 'red' ? 'Red' : 'Black';
    turnLabel.textContent = `Your move — click a ${humanSide} piece`;
  } else if (isHumanMode && gs === 'waiting_human' && !humanTurn) {
    const aiSide = humanColor === 'red' ? 'Black' : 'Red';
    turnLabel.textContent = `AI thinking (${aiSide})…`;
  } else {
    turnLabel.textContent = state.is_red_turn ? 'Red to move' : 'Black to move';
  }
  document.getElementById('move-count-label').textContent = `Move ${state.move_count}`;

  // Last move
  const hist = state.move_history;
  if (hist && hist.length > 0) {
    const last = hist[hist.length - 1];
    document.getElementById('last-move-label').textContent =
      `Last: ${last.move} (${last.is_red ? 'Red' : 'Black'})`;
    // Update lastMove highlight
    if (last.move && last.move.length >= 4) {
      const fromFile = last.move.charCodeAt(0) - 97;
      const fromRank = parseInt(last.move[1]) - 1;
      const toFile   = last.move.charCodeAt(2) - 97;
      const toRank   = parseInt(last.move[3]) - 1;
      lastMove = {
        from: coordToGridIdx(fromFile, fromRank),
        to:   coordToGridIdx(toFile,   toRank),
      };
    }
  } else {
    document.getElementById('last-move-label').textContent = '';
    lastMove = null;
  }
}

function updateFlowBanner() {
  const el = document.getElementById('flow-banner');
  const textEl = document.getElementById('flow-banner-text');
  if (!el || !textEl) return;

  const phase = getGamePhase();
  el.className = 'flow-banner phase-' + phase;

  if (state.estop_active) {
    textEl.textContent = 'E-STOP active — release E-Stop, then Start Game';
    return;
  }

  const mode = formatModeLabel(state.game_mode);
  const gs = (state.game_status || 'idle').toLowerCase();

  if (phase === 'starting') {
    textEl.textContent =
      state.game_mode === 'ai_vs_ai'
        ? `${mode}: building board, starting AI…`
        : `${mode}: building board, waiting for first move…`;
    return;
  }

  if (phase === 'setup') {
    textEl.textContent = simHint()
      ? `1) ${mode}  2) Engine Setup (right)  3) Start Game`
      : 'Press Start Game (human vs AI on physical board)';
    return;
  }
  if (phase === 'over') {
    const { title, detail } = formatGameResult(state.game_result, state.game_result_reason);
    textEl.textContent = `Finished (${mode}) — ${title}${detail ? ' · ' + detail : ''}. Change mode or New Game.`;
    return;
  }

  if (state.game_mode === 'ai_vs_ai') {
    textEl.textContent = gs === 'computing_ai'
      ? `${mode}: both sides thinking…`
      : `${mode}: game in progress`;
    return;
  }

  if (gs === 'computing_ai') {
    const aiSide = humanColor === 'red' ? 'Black' : 'Red';
    textEl.textContent = `${mode}: AI (${aiSide}) is thinking…`;
  } else if (gs === 'waiting_human') {
    const humanSide = humanColor === 'red' ? 'Red' : 'Black';
    textEl.textContent = `${mode}: your turn — click a ${humanSide} piece on the board`;
  } else if (gs === 'executing_move') {
    textEl.textContent = `${mode}: robot executing move…`;
  } else {
    textEl.textContent = `${mode}: ${formatActivity(gs)}`;
  }
}

function simHint() {
  return !!state.simulation_mode;
}

function updateGameResultBanner() {
  const banner = document.getElementById('game-result-banner');
  const titleEl = document.getElementById('game-result-title');
  const detailEl = document.getElementById('game-result-detail');
  const estopBanner = document.getElementById('estop-banner');
  const phaseChip = document.getElementById('phase-chip');

  if (estopBanner) {
    estopBanner.classList.toggle('hidden', !state.estop_active);
  }

  const phase = getGamePhase();
  if (phaseChip) {
    const labels = {
      setup: 'Setup',
      starting: 'Starting…',
      playing: 'In progress',
      over: 'Game over',
      estop: 'E-Stop',
    };
    phaseChip.textContent = labels[phase] || phase;
    phaseChip.className = 'phase-chip phase-' + phase;
  }

  if (!banner || !titleEl || !detailEl) return;

  const gs = (state.game_status || '').toLowerCase();
  const result = (state.game_result || 'ongoing').toLowerCase();
  const show = gs === 'game_over' && result !== 'ongoing';

  banner.classList.toggle('hidden', !show);
  if (!show) {
    banner.className = 'game-result-banner hidden';
    return;
  }

  const { title, detail } = formatGameResult(state.game_result, state.game_result_reason);
  titleEl.textContent = title;
  detailEl.textContent = detail;

  banner.className = 'game-result-banner';
  if (result === 'red_wins') banner.classList.add('result-red');
  else if (result === 'black_wins') banner.classList.add('result-black');
  else if (result === 'draw') banner.classList.add('result-draw');
}

function formatEngineShort(name) {
  const n = (name || '').toLowerCase();
  if (n === 'fairystockfish') return 'Stockfish';
  if (n === 'minimax') return 'Minimax';
  return name || '—';
}

function engineUsesStockfish(red, black) {
  return red === 'fairystockfish' || black === 'fairystockfish';
}

function evalScoreHint() {
  const usesMinimax =
    state.red_engine === 'minimax' || state.black_engine === 'minimax';
  if (usesMinimax) {
    return 'Centipawns (Red view). Minimax search uses heuristics; bar uses NNUE.';
  }
  return 'Centipawns (Red view). NNUE via Stockfish.';
}

function getStockfishDifficulty() {
  const slider = document.getElementById('sel-stockfish-difficulty');
  if (slider) return parseInt(slider.value, 10) || 20;
  return state.stockfish_difficulty || 20;
}

function engineMatchupLabel() {
  const red = formatEngineShort(state.red_engine);
  const black = formatEngineShort(state.black_engine);
  const skill = state.stockfish_difficulty || 20;
  const skillNote = engineUsesStockfish(state.red_engine, state.black_engine)
    ? ` · L${skill}`
    : '';
  if (state.game_mode === 'ai_vs_human') {
    return humanColor === 'red'
      ? `You (Red) vs ${black}${skillNote}`
      : `${red}${skillNote} vs You (Black)`;
  }
  return `${red} vs ${black}${skillNote}`;
}

function updateDifficultyDisplay() {
  const val = getStockfishDifficulty();
  const label = document.getElementById('stockfish-difficulty-value');
  if (label) label.textContent = String(val);
}

function updateEngineSelectors() {
  const panel = document.getElementById('panel-engines');
  const cfg = document.getElementById('engine-config');
  const selRed = document.getElementById('sel-red-engine');
  const selBlack = document.getElementById('sel-black-engine');
  const lockBadge = document.getElementById('engine-setup-lock');
  const hint = document.getElementById('engine-setup-hint');
  const matchup = document.getElementById('engine-matchup-label');
  const diffRow = document.getElementById('stockfish-difficulty-row');
  const diffSlider = document.getElementById('sel-stockfish-difficulty');
  if (!cfg || !selRed || !selBlack) return;

  const sim = !!state.simulation_mode;
  const humanMode = state.game_mode === 'ai_vs_human';
  const locked = !canChangeMode() || isGameStarting() || isGameInProgress();
  const humanIsRed = humanColor === 'red';
  const redVal = selRed.value;
  const blackVal = selBlack.value;
  const showDifficulty = humanMode
    ? engineUsesStockfish(humanIsRed ? blackVal : redVal, humanIsRed ? blackVal : redVal)
    : engineUsesStockfish(redVal, blackVal);

  if (panel) panel.classList.remove('hidden');

  // Color picker — only in human mode
  const colorRow = document.getElementById('human-color-row');
  if (colorRow) colorRow.classList.toggle('hidden', !humanMode);

  // Per-side visibility: human side shows "(you)", AI side shows selector
  const youRedEl  = document.getElementById('engine-you-red');
  const youBlackEl = document.getElementById('engine-you-black');
  if (humanMode) {
    selRed.classList.toggle('hidden', humanIsRed);
    selBlack.classList.toggle('hidden', !humanIsRed);
    if (youRedEl)   youRedEl.classList.toggle('hidden', !humanIsRed);
    if (youBlackEl) youBlackEl.classList.toggle('hidden', humanIsRed);
  } else {
    selRed.classList.remove('hidden');
    selBlack.classList.remove('hidden');
    if (youRedEl)   youRedEl.classList.add('hidden');
    if (youBlackEl) youBlackEl.classList.add('hidden');
  }

  selRed.disabled   = locked || (humanMode && humanIsRed);
  selBlack.disabled = locked || (humanMode && !humanIsRed);
  if (diffSlider) diffSlider.disabled = locked;

  if (diffRow) diffRow.classList.toggle('hidden', !showDifficulty);
  if (diffSlider && state.stockfish_difficulty) {
    diffSlider.value = String(state.stockfish_difficulty);
  }
  updateDifficultyDisplay();

  if (lockBadge) lockBadge.classList.toggle('hidden', !locked);
  if (matchup) matchup.textContent = engineMatchupLabel();

  if (state.red_engine && selRed.value !== state.red_engine) selRed.value = state.red_engine;
  if (state.black_engine && selBlack.value !== state.black_engine) selBlack.value = state.black_engine;
}

function updateAIPanel() {
  const rawCp = state.evaluation_cp || 0;
  const cp = clampEvalCp(rawCp);
  const pawns = cp != null ? (cp / 100).toFixed(2) : '—';

  const gs = (state.game_status || '').toLowerCase();
  let engineLabel = engineMatchupLabel();
  if (gs === 'computing_ai') {
    const sideEngine = state.is_red_turn ? state.red_engine : state.black_engine;
    const side = state.is_red_turn ? 'Red' : 'Black';
    if (state.game_mode === 'ai_vs_human' && state.is_red_turn) {
      engineLabel = 'Waiting for you';
    } else {
      engineLabel = `${side} · ${formatEngineShort(sideEngine)}`;
    }
  }
  document.getElementById('engine-badge').textContent = engineLabel;

  const barVal = cp != null ? cp : 0;
  const clampBar = Math.max(-1500, Math.min(1500, barVal));
  const redPct  = Math.round((clampBar + 1500) / 30);
  const blackPct = 100 - redPct;
  document.getElementById('eval-bar-red').style.width   = redPct + '%';
  document.getElementById('eval-bar-black').style.width = blackPct + '%';
  document.getElementById('eval-label').textContent =
    cp != null ? ((cp >= 0 ? '+' : '') + pawns) : '—';
  const hintEl = document.getElementById('eval-score-hint');
  if (hintEl) hintEl.textContent = evalScoreHint();

  document.getElementById('ai-eval').textContent =
    cp != null ? ((cp >= 0 ? '+' : '') + cp + ' cp') : (Math.abs(rawCp) > 50000 ? 'Mate/score' : '—');
  document.getElementById('ai-depth').textContent    = state.depth_reached || '--';
  document.getElementById('ai-time').textContent     = (state.thinking_time || 0) + ' s';
  document.getElementById('ai-bestmove').textContent = state.best_move || '--';
  document.getElementById('ai-ponder').textContent   = state.ponder_move || '--';
}

function updateSystemPanel() {
  const phase = getGamePhase();
  const phaseLabels = {
    setup: 'Setup',
    starting: 'Starting…',
    playing: 'In progress',
    over: 'Game over',
    estop: 'E-Stop',
  };
  const phaseEl = document.getElementById('sys-phase');
  if (phaseEl) phaseEl.textContent = phaseLabels[phase] || phase;

  const gsEl = document.getElementById('sys-game-state');
  if (gsEl) {
    gsEl.textContent = phase === 'starting'
      ? 'Starting game…'
      : formatActivity(state.game_status);
  }

  const modeEl = document.getElementById('sys-game-mode');
  if (modeEl) modeEl.textContent = formatModeLabel(state.game_mode);

  const grEl = document.getElementById('sys-game-result');
  if (grEl) {
    const phaseSetup = phase === 'setup' && (state.move_count || 0) === 0;
    if (phaseSetup) {
      grEl.textContent = 'Not started';
      grEl.className = 'result-ongoing';
    } else if (phase === 'starting') {
      grEl.textContent = 'Starting…';
      grEl.className = 'result-ongoing';
    } else if ((state.game_result || 'ongoing') === 'ongoing') {
      grEl.textContent = 'In progress';
      grEl.className = 'result-ongoing';
    } else {
      const { title, detail, css } = formatGameResult(
        state.game_result, state.game_result_reason
      );
      grEl.textContent = detail ? `${title} (${detail})` : title;
      grEl.className = css || 'status-warn';
    }
  }

  document.getElementById('sys-move-count').textContent = state.move_count || 0;
  document.getElementById('sys-gripper').textContent    = state.gripper_active ? 'Active' : 'Idle';

  const estopEl = document.getElementById('sys-estop');
  estopEl.textContent = state.estop_active ? 'ACTIVE' : 'OK';
  estopEl.className   = state.estop_active ? 'status-err' : 'status-ok';

  const estopBtn = document.getElementById('btn-estop');
  const estopHint = document.getElementById('estop-hint');
  if (estopBtn) {
    estopBtn.classList.toggle('active', state.estop_active);
    estopBtn.textContent = state.estop_active ? 'Release E-Stop' : 'E-Stop';
  }
  if (estopHint) {
    estopHint.textContent = state.estop_active ? 'Click to resume' : 'Halt robot / AI';
  }
}

function updateHistory() {
  const list = document.getElementById('history-list');
  const note = document.getElementById('history-count-note');
  const hist = state.move_history || [];
  const total = state.move_count || hist.length;

  // Only re-render when count changes (optimisation)
  if (list.dataset.count === String(hist.length) && list.dataset.total === String(total)) return;
  list.dataset.count = hist.length;
  list.dataset.total = total;

  if (note) {
    if (total > hist.length) {
      note.textContent = `last ${hist.length} of ${total}`;
      note.classList.remove('hidden');
    } else {
      note.textContent = '';
      note.classList.add('hidden');
    }
  }

  list.innerHTML = '';
  const startNum = total - hist.length + 1;
  hist.slice().reverse().forEach((entry, i) => {
    const num  = total - i;
    const side = entry.is_red ? 'red' : 'black';
    const div  = document.createElement('div');
    div.className = 'history-entry';
    const eng = (entry.engine || '').toLowerCase();
    const engTag = eng.includes('fairy') || eng.includes('stock')
      ? 'SF'
      : eng.includes('minimax')
        ? 'MM'
        : eng === 'human'
          ? 'H'
          : '';
    div.innerHTML = `
      <span class="history-num">${num}.</span>
      <span class="history-dot ${side}"></span>
      <span class="history-move ${side}-move">${entry.move}</span>
      <span class="history-meta">${engTag ? engTag + ' ' : ''}${entry.depth ? 'd' + entry.depth : ''}
        ${entry.eval != null ? (entry.eval >= 0 ? '+' : '') + entry.eval + 'cp' : ''}
        ${entry.time ? entry.time + 's' : ''}
      </span>
    `;
    list.appendChild(div);
  });
}

// ── Canvas Interaction (AI vs Human) ──────────────────────────────

canvas.addEventListener('click', (e) => {
  if (!state.simulation_mode) return;
  if (state.estop_active) {
    showToast('Release E-Stop before playing');
    return;
  }
  if (state.game_mode !== 'ai_vs_human') return;
  if ((state.game_status || '').toLowerCase() !== 'waiting_human') return;
  const humanIsRed2 = humanColor === 'red';
  if (humanIsRed2 ? !state.is_red_turn : state.is_red_turn) return;

  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width  / rect.width;
  const scaleY = canvas.height / rect.height;
  const px = (e.clientX - rect.left)  * scaleX;
  const py = (e.clientY - rect.top)   * scaleY;

  const clickedIdx = pixelToGridIdx(px, py);
  if (clickedIdx === null) return;

  const clickedCode = state.board_grid[clickedIdx];

  if (selectedIdx === null) {
    // Select a piece belonging to the human's color (positive = red, negative = black)
    if (humanIsRed2 ? clickedCode > 0 : clickedCode < 0) {
      selectedIdx = clickedIdx;
      fetchLegalDests(clickedIdx);
    }
  } else {
    if (clickedIdx === selectedIdx) {
      // Deselect
      selectedIdx = null;
      legalDests  = [];
      drawBoard();
    } else if (clickedCode > 0) {
      // Select different Red piece
      selectedIdx = clickedIdx;
      fetchLegalDests(clickedIdx);
    } else if (legalDests.includes(clickedIdx)) {
      // Make the move
      const { file: fromFile, rank: fromRank } = gridIdxToCoord(selectedIdx);
      const { file: toFile,   rank: toRank   } = gridIdxToCoord(clickedIdx);
      const moveUCI = coordToUCI(fromFile, fromRank) + coordToUCI(toFile, toRank);
      submitMove(moveUCI);
      selectedIdx = null;
      legalDests  = [];
    } else {
      // Click elsewhere — deselect
      selectedIdx = null;
      legalDests  = [];
      drawBoard();
    }
  }
});

function fetchLegalDests(fromIdx) {
  const { file, rank } = gridIdxToCoord(fromIdx);
  const fromUCI = coordToUCI(file, rank);
  legalDests = [];
  drawBoard();

  const reqId = ++legalDestsRequestId;
  fetch('/api/legal_moves', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ from: fromUCI }),
  })
    .then((r) => r.json())
    .then((d) => {
      if (reqId !== legalDestsRequestId || selectedIdx !== fromIdx) return;
      if (!d.ok) {
        if (d.error) showToast(d.error);
        legalDests = [];
      } else {
        legalDests = Array.isArray(d.dest_indices) ? d.dest_indices : [];
      }
      drawBoard();
    })
    .catch(() => {
      if (reqId !== legalDestsRequestId) return;
      legalDests = [];
      showToast('Could not load legal moves');
      drawBoard();
    });
}

function submitMove(moveUCI) {
  fetch('/api/simulate_move', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ move: moveUCI }),
  })
  .then(r => r.json())
  .then(d => {
    if (!d.ok) showToast('Move rejected: ' + (d.error || 'unknown'));
  })
  .catch(() => showToast('Failed to submit move'));
}

// ── Mode Switching ────────────────────────────────────────────────

function publishEngineSetup() {
  const selRed = document.getElementById('sel-red-engine');
  const selBlack = document.getElementById('sel-black-engine');
  if (!selRed || !selBlack) return Promise.resolve();
  const red = selRed.value;
  const black = selBlack.value;
  const body = {
    red_engine: red,
    black_engine: black,
    stockfish_difficulty: getStockfishDifficulty(),
  };
  return fetch('/api/set_engines', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
}

function onDifficultyInput() {
  updateDifficultyDisplay();
}

function onDifficultyChange() {
  if (!canChangeMode() || isGameStarting() || isGameInProgress()) {
    updateEngineSelectors();
    return;
  }
  const skill = getStockfishDifficulty();
  state.stockfish_difficulty = skill;
  publishEngineSetup()
    .then((r) => r.json())
    .then((d) => {
      if (d.ok) {
        if (d.stockfish_difficulty != null) {
          state.stockfish_difficulty = d.stockfish_difficulty;
        }
        renderAll();
      } else if (d.error) {
        showToast(d.error);
        updateEngineSelectors();
      }
    })
    .catch(() => showToast('Failed to set Stockfish level'));
}

function onEngineChange() {
  if (!canChangeMode() || isGameStarting() || isGameInProgress()) {
    updateEngineSelectors();
    return;
  }
  const selRed = document.getElementById('sel-red-engine');
  const selBlack = document.getElementById('sel-black-engine');
  if (!selRed || !selBlack) return;
  const red = selRed.value;
  const black = selBlack.value;
  publishEngineSetup()
  .then(r => r.json())
  .then(d => {
    if (d.ok) {
      state.red_engine = d.red_engine || red;
      state.black_engine = d.black_engine || black;
      if (d.stockfish_difficulty != null) {
        state.stockfish_difficulty = d.stockfish_difficulty;
      }
      renderAll();
    } else if (d.error) {
      showToast(d.error);
      updateEngineSelectors();
    }
  })
  .catch(() => showToast('Failed to set engines'));
}

function setMode(mode) {
  if (!canChangeMode()) {
    showToast('Mode is locked during a game. Wait for game over or finish the current game.');
    return;
  }
  if (!state.simulation_mode && mode === 'ai_vs_ai') {
    showToast('AI vs AI is only available in simulation mode');
    return;
  }
  fetch('/api/set_mode', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mode }),
  })
  .then(r => r.json())
  .then(d => {
    if (d.ok) {
      state.game_mode = mode;
      selectedIdx = null;
      legalDests  = [];
      renderAll();
    } else if (d.error) {
      showToast(d.error);
    }
  })
  .catch(() => showToast('Failed to change game mode'));
}

// ── Game Controls ─────────────────────────────────────────────────

function newGame() {
  if (state.estop_active) {
    showToast('Release E-Stop first');
    return;
  }
  if (isGameInProgress()) {
    showToast('Game in progress — use Stop or Reset');
    return;
  }
  selectedIdx = null;
  legalDests  = [];
  lastMove    = null;
  beginGameStart('Starting game…');
  fetch('/api/new_game', { method: 'POST' })
    .then(r => r.json().then(d => ({ ok: r.ok, d })))
    .then(({ ok, d }) => {
      if (!ok || (d && !d.ok)) {
        endGameStart();
        showToast((d && d.error) || 'Start failed');
        renderAll();
      }
    })
    .catch(() => {
      endGameStart();
      showToast('Failed to start game');
      renderAll();
    });
}

function applyIdleUiState() {
  state.game_status = 'idle';
  state.move_count = 0;
  state.move_history = [];
  state.game_result = 'ongoing';
  state.game_result_reason = '';
  state.best_move = '';
  state.ponder_move = '';
  state.evaluation_cp = 0;
  state.depth_reached = 0;
  state.thinking_time = 0;
  state.is_red_turn = true;
}

function stopGame() {
  if (!isGameInProgress() && !isGameStarting()) return;
  endGameStart();
  selectedIdx = null;
  legalDests  = [];
  lastMove    = null;
  applyIdleUiState();
  renderAll();
  fetch('/api/stop_game', { method: 'POST' })
    .then(() => {
      showToast('Stopped — board reset. Press Start for a new game.');
      renderAll();
    })
    .catch(() => showToast('Failed to stop game'));
}

function resetGame() {
  if (state.estop_active) {
    showToast('Release E-Stop first');
    return;
  }
  selectedIdx = null;
  legalDests  = [];
  lastMove    = null;
  state.move_history = [];
  state.move_count = 0;
  beginGameStart('Restarting game…');
  fetch('/api/reset_game', { method: 'POST' })
    .then(r => r.json().then(d => ({ ok: r.ok, d })))
    .then(({ ok, d }) => {
      if (!ok || (d && !d.ok)) {
        endGameStart();
        showToast((d && d.error) || 'Reset failed');
        renderAll();
      }
    })
    .catch(() => {
      endGameStart();
      showToast('Failed to reset game');
      renderAll();
    });
}

let estopActive = false;
function toggleEstop() {
  const next = !state.estop_active;
  fetch('/api/emergency_stop', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ active: next }),
  })
  .then(() => {
    estopActive = next;
    state.estop_active = next;
    if (next) {
      endGameStart();
      selectedIdx = null;
      legalDests = [];
      showToast('E-STOP engaged — release when safe, then New Game');
    } else {
      showToast('E-Stop released — press New Game to continue');
    }
    renderAll();
  })
  .catch(() => showToast('Failed to toggle E-Stop'));
}

// ── Toast ─────────────────────────────────────────────────────────

let toastTimer = null;
function showToast(msg) {
  const toast = document.getElementById('alert-toast');
  toast.textContent = msg;
  toast.classList.remove('hidden');
  toast.classList.add('visible');
  if (toastTimer) clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    toast.classList.remove('visible');
    setTimeout(() => toast.classList.add('hidden'), 300);
  }, 3500);
}

// ── REST sync during fast AI play (WebSocket alone can fall behind) ─

let syncPollTimer = null;
let syncPollIntervalMs = 0;
const SYNC_POLL_ACTIVE_MS = 350;
const SYNC_POLL_DISCONNECTED_MS = 800;

function updateSyncPolling() {
  const active = isGameStarting() || isGameInProgress();
  const interval = active
    ? (socket.connected ? SYNC_POLL_ACTIVE_MS : SYNC_POLL_DISCONNECTED_MS)
    : 0;

  if (interval === syncPollIntervalMs && syncPollTimer) return;
  syncPollIntervalMs = interval;

  if (syncPollTimer) {
    clearInterval(syncPollTimer);
    syncPollTimer = null;
  }
  if (!interval) return;

  syncPollTimer = setInterval(() => {
    fetchStateSnapshot().catch(() => {});
  }, interval);
}

socket.on('connect_error', () => {
  updateSyncPolling();
});

// ── Initial draw ──────────────────────────────────────────────────

// Draw empty board immediately while waiting for socket
drawBoard();

// Also fetch state immediately via REST as fallback
fetchStateSnapshot().catch(() => {});
