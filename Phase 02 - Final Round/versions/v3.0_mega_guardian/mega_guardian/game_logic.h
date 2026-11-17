/*
 * game_logic.h  -  GuardianBot v3 (Arduino Mega)  -  pure-C mission logic
 *
 * Everything in here is plain C with no Arduino dependency so the exact same
 * code is compiled and unit-tested on a PC (firmware/test/test_logic.c) and
 * then flashed unchanged to the Mega.
 *
 *   Section 2:  maze map (9 x 4 cells), BFS shortest path (fewest cells, then
 *               fewest turns), right/left turn counting per rulebook 3.2
 *   Section 3:  Lo Shu board numbering, clue -> piece mapping, winning move
 *               (rulebook 3.3, prefers horizontal rows as recommended)
 */
#ifndef GAME_LOGIC_H
#define GAME_LOGIC_H

#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------------ maze */
#define MZ_COLS 9
#define MZ_ROWS 4

/* heading index: 0 = +Y (deeper into the maze, away from Section 1),
 *                1 = +X (to the robot's right at entry), 2 = -Y, 3 = -X   */
#define H_N 0
#define H_E 1
#define H_S 2
#define H_W 3

#define W_UNKNOWN 0
#define W_OPEN    1
#define W_WALL    2

#define TILE_NONE  0
#define TILE_BLUE  1
#define TILE_GREEN 2

typedef struct {
    uint8_t wall[MZ_COLS][MZ_ROWS][4];   /* W_UNKNOWN / W_OPEN / W_WALL per side */
    uint8_t visited[MZ_COLS][MZ_ROWS];
    uint8_t tile[MZ_COLS][MZ_ROWS];      /* TILE_* */
} MazeMap;

static const int8_t MZ_DX[4] = { 0, 1, 0, -1 };
static const int8_t MZ_DY[4] = { 1, 0, -1, 0 };

static inline int mz_in(int c, int r) { return c >= 0 && c < MZ_COLS && r >= 0 && r < MZ_ROWS; }

static inline void mz_init(MazeMap *m)
{
    memset(m, 0, sizeof(*m));
    /* outer boundary is always a wall */
    for (int c = 0; c < MZ_COLS; c++) { m->wall[c][0][H_S] = W_WALL; m->wall[c][MZ_ROWS - 1][H_N] = W_WALL; }
    for (int r = 0; r < MZ_ROWS; r++) { m->wall[0][r][H_W] = W_WALL; m->wall[MZ_COLS - 1][r][H_E] = W_WALL; }
}

/* set one wall and its mirror in the neighbouring cell */
static inline void mz_set_wall(MazeMap *m, int c, int r, int h, uint8_t state)
{
    if (!mz_in(c, r)) return;
    m->wall[c][r][h] = state;
    int nc = c + MZ_DX[h], nr = r + MZ_DY[h];
    if (mz_in(nc, nr)) m->wall[nc][nr][(h + 2) & 3] = state;
}

static inline int mz_open_count(const MazeMap *m, int c, int r)
{
    int n = 0;
    for (int h = 0; h < 4; h++) if (m->wall[c][r][h] == W_OPEN) n++;
    return n;
}

static inline int mz_is_dead_end(const MazeMap *m, int c, int r)
{
    int walls = 0, unknown = 0;
    for (int h = 0; h < 4; h++) {
        if (m->wall[c][r][h] == W_WALL) walls++;
        else if (m->wall[c][r][h] == W_UNKNOWN) unknown++;
    }
    return walls == 3 && unknown == 0;
}

/* count blue / green tiles recorded in the map */
static inline void mz_tile_counts(const MazeMap *m, int *blue, int *green)
{
    *blue = 0; *green = 0;
    for (int c = 0; c < MZ_COLS; c++)
        for (int r = 0; r < MZ_ROWS; r++) {
            if (m->tile[c][r] == TILE_BLUE)  (*blue)++;
            if (m->tile[c][r] == TILE_GREEN) (*green)++;
        }
}

/* ---------------------------------------------------------- BFS path --- */
/* State = (cell, heading).  Cost = (cells, turns) lexicographic.            *
 * Only W_OPEN edges are used (unknown edges are treated as walls) unless    *
 * allow_unknown != 0, which is used during exploration to reach frontiers.  *
 * Result: path of headings (one per move) in out_h[], length returned.      *
 * Rulebook: turns inside the final (exit) cell are NOT counted.             */
#define MZ_STATES (MZ_COLS * MZ_ROWS * 4)
#define MZ_MAXPATH 64

typedef struct {
    int len;                 /* number of moves                               */
    uint8_t h[MZ_MAXPATH];   /* heading of each move                          */
    int right_turns, left_turns;
    int found;
} MazePath;

static inline int mz_sid(int c, int r, int h) { return (c * MZ_ROWS + r) * 4 + h; }

static inline int mz_plan(const MazeMap *m, int c0, int r0, int h0, int gc, int gr,
                          int allow_unknown, MazePath *out)
{
    /* Dijkstra with small integer cost: cost = cells*16 + turns (turns < 16 per path is safe) */
    static uint16_t dist[MZ_STATES];
    static int16_t  prev[MZ_STATES];
    static uint8_t  done[MZ_STATES];
    for (int i = 0; i < MZ_STATES; i++) { dist[i] = 0xFFFF; prev[i] = -1; done[i] = 0; }
    int s0 = mz_sid(c0, r0, h0);
    dist[s0] = 0;
    out->found = 0; out->len = 0; out->right_turns = out->left_turns = 0;
    if (c0 == gc && r0 == gr) { out->found = 1; return 1; }

    for (;;) {
        /* pick cheapest open state (array scan is fine for 144 states) */
        int best = -1; uint16_t bd = 0xFFFF;
        for (int i = 0; i < MZ_STATES; i++)
            if (!done[i] && dist[i] < bd) { bd = dist[i]; best = i; }
        if (best < 0) return 0;
        done[best] = 1;
        int h = best & 3, cell = best >> 2, c = cell / MZ_ROWS, r = cell % MZ_ROWS;
        if (c == gc && r == gr) {
            /* reconstruct */
            int cur = best, n = 0;
            static uint8_t tmp[MZ_MAXPATH];
            while (prev[cur] >= 0 && n < MZ_MAXPATH) { tmp[n++] = cur & 3; cur = prev[cur]; }
            out->len = n;
            for (int i = 0; i < n; i++) out->h[i] = tmp[n - 1 - i];
            /* turn counting: heading changes between consecutive moves, plus the
             * first move relative to the entry heading.  The turn made in the
             * exit cell is excluded automatically because no move follows it. */
            int ph = h0;
            for (int i = 0; i < n; i++) {
                int d = (out->h[i] - ph + 4) & 3;
                if (d == 1) out->right_turns++;
                else if (d == 3) out->left_turns++;
                ph = out->h[i];
            }
            out->found = 1;
            return 1;
        }
        for (int h2 = 0; h2 < 4; h2++) {
            uint8_t w = m->wall[c][r][h2];
            if (w == W_WALL) continue;
            if (w == W_UNKNOWN && !allow_unknown) continue;
            if (h2 == ((h + 2) & 3) && !(c == c0 && r == r0)) continue;   /* no U-turn mid-path */
            int nc = c + MZ_DX[h2], nr = r + MZ_DY[h2];
            if (!mz_in(nc, nr)) continue;
            uint16_t nd = dist[best] + 16 + (h2 != h ? 1 : 0) + (w == W_UNKNOWN ? 2 : 0);
            int ns = mz_sid(nc, nr, h2);
            if (nd < dist[ns]) { dist[ns] = nd; prev[ns] = best; }
        }
    }
}

/* Nearest cell that still has an UNKNOWN side (exploration frontier).      *
 * Returns 1 and fills gc/gr, or 0 when the maze is fully explored.         */
static inline int mz_nearest_frontier(const MazeMap *m, int c0, int r0, int h0, int *gc, int *gr)
{
    int bestcost = 0x7FFF, bc = -1, br = -1;
    MazePath p;
    for (int c = 0; c < MZ_COLS; c++)
        for (int r = 0; r < MZ_ROWS; r++) {
            if (m->visited[c][r]) continue;
            /* reachable through known-open or unknown edges? */
            int touch = 0;
            for (int h = 0; h < 4; h++) if (m->wall[c][r][h] != W_WALL) touch = 1;
            if (!touch) continue;
            if (!mz_plan(m, c0, r0, h0, c, r, 1, &p)) continue;
            int cost = p.len * 16 + p.right_turns + p.left_turns;
            if (cost < bestcost) { bestcost = cost; bc = c; br = r; }
        }
    if (bc < 0) return 0;
    *gc = bc; *gr = br;
    return 1;
}

/* --------------------------------------------------------- Lo Shu board */
/* MAGIC[row][col]: row 0 = entrance row, col 0 = robot's LEFT seen from the
 * entrance.  Position 1 = middle of the first row, 6 on the left.
 *      6 1 8
 *      7 5 3
 *      2 9 4     (every row / column / diagonal sums to 15)              */
static const uint8_t MAGIC[3][3] = { {6, 1, 8}, {7, 5, 3}, {2, 9, 4} };

#define CLAN_BLUE  1
#define CLAN_GREEN 2

typedef struct {
    uint8_t piece[10];      /* index 1..9 : 0 empty, CLAN_BLUE, CLAN_GREEN */
    uint8_t unresolved_blue, unresolved_green;   /* clue values that could not be placed */
} Board;

static inline void board_row_col(int pos, int *row, int *col)
{
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) if (MAGIC[r][c] == pos) { *row = r; *col = c; return; }
    *row = *col = -1;
}

/* Apply the four clues.  Returns clan (majority colour).                   *
 * Blue tiles -> Cylinder 1 (blue), green tiles -> Cube 1 (green),         *
 * right turns -> Cylinder 2 (blue), left turns -> Cube 2 (green).          *
 * A value outside 1..9 or already used cannot be placed and is counted as *
 * unresolved (the robot then inspects the board, rulebook 3.3).            */
static inline int board_from_clues(Board *b, int blue_tiles, int green_tiles, int right_turns, int left_turns)
{
    memset(b, 0, sizeof(*b));
    const int v[4]   = { blue_tiles, green_tiles, right_turns, left_turns };
    const uint8_t col[4] = { CLAN_BLUE, CLAN_GREEN, CLAN_BLUE, CLAN_GREEN };
    for (int i = 0; i < 4; i++) {
        if (v[i] >= 1 && v[i] <= 9 && b->piece[v[i]] == 0) b->piece[v[i]] = col[i];
        else if (col[i] == CLAN_BLUE) b->unresolved_blue++;
        else b->unresolved_green++;
    }
    return blue_tiles > green_tiles ? CLAN_BLUE : CLAN_GREEN;
}

/* Lines in rulebook-preferred order: horizontal rows first (more turning   *
 * room), then columns, then diagonals.  Rows here are rows of the board    *
 * as seen from the entrance, i.e. lines ACROSS the robot's travel axis.   *
 * NOTE: the sim's "along" set = columns of MAGIC.  The rulebook says a    *
 * horizontal row gives more space, and on Figure 1 the board's 40 cm     *
 * pitch is along the robot's travel direction (positions 6-1-8 are a    *
 * row across).  Preference order is configurable via BOARD_PREFER_ROWS.  */
#ifndef BOARD_PREFER_ROWS
#define BOARD_PREFER_ROWS 1
#endif

static inline int board_winning_move(const Board *b, int clan, uint8_t line_out[3])
{
    uint8_t lines[8][3];
    int n = 0;
    if (BOARD_PREFER_ROWS) {
        for (int r = 0; r < 3; r++) { lines[n][0] = MAGIC[r][0]; lines[n][1] = MAGIC[r][1]; lines[n][2] = MAGIC[r][2]; n++; }
        for (int c = 0; c < 3; c++) { lines[n][0] = MAGIC[0][c]; lines[n][1] = MAGIC[1][c]; lines[n][2] = MAGIC[2][c]; n++; }
    } else {
        for (int c = 0; c < 3; c++) { lines[n][0] = MAGIC[0][c]; lines[n][1] = MAGIC[1][c]; lines[n][2] = MAGIC[2][c]; n++; }
        for (int r = 0; r < 3; r++) { lines[n][0] = MAGIC[r][0]; lines[n][1] = MAGIC[r][1]; lines[n][2] = MAGIC[r][2]; n++; }
    }
    lines[n][0] = 6; lines[n][1] = 5; lines[n][2] = 4; n++;
    lines[n][0] = 8; lines[n][1] = 5; lines[n][2] = 2; n++;

    for (int i = 0; i < n; i++) {
        int mine = 0, empty = -1, emptyn = 0;
        for (int k = 0; k < 3; k++) {
            int p = lines[i][k];
            if (b->piece[p] == clan) mine++;
            else if (b->piece[p] == 0) { empty = p; emptyn++; }
        }
        if (mine == 2 && emptyn == 1) {
            if (line_out) { line_out[0] = lines[i][0]; line_out[1] = lines[i][1]; line_out[2] = lines[i][2]; }
            return empty;
        }
    }
    return 0;   /* no winning move */
}

/* Fallback when no line can be completed: block the opponent's winning   *
 * move if there is one, else take the centre, else first free position.  */
static inline int board_fallback_move(const Board *b, int clan)
{
    int opp = (clan == CLAN_BLUE) ? CLAN_GREEN : CLAN_BLUE;
    int p = board_winning_move(b, opp, 0);
    if (p) return p;
    if (b->piece[5] == 0) return 5;
    static const uint8_t order[9] = { 1, 7, 3, 9, 6, 8, 2, 4, 5 };
    for (int i = 0; i < 9; i++) if (b->piece[order[i]] == 0) return order[i];
    return 0;
}

#endif /* GAME_LOGIC_H */
