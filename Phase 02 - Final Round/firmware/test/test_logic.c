/* Host unit test for game_logic.h   (gcc -I../mega_guardian test_logic.c -o test_logic && ./test_logic) */
#include <stdio.h>
#include <stdlib.h>
#include "game_logic.h"

static int fails = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); fails++; } else printf("ok   %s\n", msg); } while (0)

/* Build a maze from an ASCII picture (rows listed top = r3 ... bottom = r0).
 * Each cell is 2 chars wide, walls drawn with | and -, like:
 *   +-+-+
 *   | | |
 *   +-+ +      */
static void load_ascii(MazeMap *m, const char *rows[]) {
    /* rows[] has 2*MZ_ROWS+1 strings, each 2*MZ_COLS+1 chars; rows[0] is the top (r = MZ_ROWS-1 north wall) */
    mz_init(m);
    for (int c = 0; c < MZ_COLS; c++)
        for (int r = 0; r < MZ_ROWS; r++) {
            int y = 2 * (MZ_ROWS - 1 - r) + 1, x = 2 * c + 1;
            mz_set_wall(m, c, r, H_N, rows[y - 1][x] == '-' ? W_WALL : W_OPEN);
            mz_set_wall(m, c, r, H_S, rows[y + 1][x] == '-' ? W_WALL : W_OPEN);
            mz_set_wall(m, c, r, H_W, rows[y][x - 1] == '|' ? W_WALL : W_OPEN);
            mz_set_wall(m, c, r, H_E, rows[y][x + 1] == '|' ? W_WALL : W_OPEN);
        }
}

int main(void) {
    /* ---- Test 1: straight corridor, no turns */
    MazeMap m; mz_init(&m);
    for (int c = 0; c < MZ_COLS; c++) for (int r = 0; r < MZ_ROWS; r++) for (int h = 0; h < 4; h++)
        if (m.wall[c][r][h] == W_UNKNOWN) mz_set_wall(&m, c, r, h, W_OPEN);
    MazePath p;
    CHECK(mz_plan(&m, 0, 0, H_N, 0, 3, 0, &p) && p.len == 3 && p.right_turns == 0 && p.left_turns == 0, "open maze straight north: 3 moves, 0 turns");
    CHECK(mz_plan(&m, 0, 0, H_N, 8, 3, 0, &p) && p.len == 11 && (p.right_turns + p.left_turns) == 1, "open maze to (8,3): 11 moves, exactly 1 turn (fewest turns tie-break)");
    CHECK(p.right_turns == 1 || p.left_turns == 1, "the single turn is counted as R or L");

    /* ---- Test 2: a real maze picture; entry (0,0) heading N, exit (8,3) */
    const char *pic[] = {
        "+-+-+-+-+-+-+-+-+-+",   /* r3 north wall */
        "|       |     |    ",   /* r3            exit at right of (8,3) -> open east side drawn as ' ' */
        "+ +-+-+ + +-+ + +-+",
        "| |   | | |   |   |",   /* r2 */
        "+ + + + + +   +-+ +",
        "| | | |   | |     |",   /* r1 */
        "+ +-+ +-+-+ + +-+ +",
        "|   |       |   | |",   /* r0 */
        "+-+-+-+-+-+-+-+-+-+",
    };
    load_ascii(&m, pic);
    CHECK(mz_plan(&m, 0, 0, H_N, 8, 3, 0, &p), "picture maze: path to exit exists");
    printf("     path len=%d R=%d L=%d :", p.len, p.right_turns, p.left_turns);
    for (int i = 0; i < p.len; i++) printf(" %c", "NESW"[p.h[i]]);
    printf("\n");
    /* hand check: (0,0)N->(0,1)N->(0,2)N->(0,3) E->(1,3)E->(2,3)E->(3,3) S->(3,2)S->(3,1) E->(4,1)N->(4,2)N->(4,3) E->(5,3)...
       the planner must find a legal path; verify by walking it */
    int c = 0, r = 0, ok = 1;
    for (int i = 0; i < p.len; i++) { if (m.wall[c][r][p.h[i]] != W_OPEN) ok = 0; c += MZ_DX[p.h[i]]; r += MZ_DY[p.h[i]]; }
    CHECK(ok && c == 8 && r == 3, "picture maze: path only crosses open walls and ends at the exit");
    /* turn count = number of heading changes starting from H_N */
    int ph = H_N, R = 0, L = 0;
    for (int i = 0; i < p.len; i++) { int d = (p.h[i] - ph + 4) & 3; if (d == 1) R++; if (d == 3) L++; ph = p.h[i]; }
    CHECK(R == p.right_turns && L == p.left_turns, "turn counts match an independent recount");

    /* dead ends */
    int de = 0;
    for (c = 0; c < MZ_COLS; c++) for (r = 0; r < MZ_ROWS; r++) if (mz_is_dead_end(&m, c, r)) de++;
    printf("     dead ends in picture maze: %d\n", de);
    CHECK(de >= 3, "picture maze has dead ends");

    /* frontier search on a fresh map: nearest frontier from entry must be entry's own neighbour */
    MazeMap f; mz_init(&f);
    f.visited[0][0] = 1;
    mz_set_wall(&f, 0, 0, H_N, W_OPEN); mz_set_wall(&f, 0, 0, H_E, W_WALL);
    int gc, gr;
    CHECK(mz_nearest_frontier(&f, 0, 0, H_N, &gc, &gr) && gc == 0 && gr == 1, "nearest frontier is the open neighbour (0,1)");
    /* fully explored = no frontier */
    for (c = 0; c < MZ_COLS; c++) for (r = 0; r < MZ_ROWS; r++) f.visited[c][r] = 1;
    CHECK(!mz_nearest_frontier(&f, 0, 0, H_N, &gc, &gr), "no frontier when every cell is visited");

    /* ---- Test 3: board logic */
    Board b;
    int clan = board_from_clues(&b, 3, 2, 5, 4);        /* sample from the sim docs */
    CHECK(clan == CLAN_BLUE, "3 blue vs 2 green -> Blue Sentinels");
    CHECK(b.piece[3] == CLAN_BLUE && b.piece[2] == CLAN_GREEN && b.piece[5] == CLAN_BLUE && b.piece[4] == CLAN_GREEN, "clues 3/2/5/4 placed");
    CHECK(b.unresolved_blue == 0 && b.unresolved_green == 0, "no unresolved pieces");
    uint8_t line[3];
    int win = board_winning_move(&b, clan, line);
    CHECK(win == 7, "blue at 3 and 5 -> winning move 7 (row 7-5-3)");

    clan = board_from_clues(&b, 3, 2, 3, 3);           /* duplicate values: the sim's actual maze */
    CHECK(b.unresolved_blue == 1 && b.unresolved_green == 1, "3/2/3/3 -> one blue and one green unresolved");
    /* after inspection the robot finds pieces at 4 (green) and 5 (blue) */
    b.piece[5] = CLAN_BLUE; b.piece[4] = CLAN_GREEN;
    win = board_winning_move(&b, CLAN_BLUE, line);
    CHECK(win == 7, "board G.G/.BB/... -> winning move 7");

    clan = board_from_clues(&b, 1, 4, 2, 6);           /* green majority */
    CHECK(clan == CLAN_GREEN, "1 blue vs 4 green -> Green Keepers");
    win = board_winning_move(&b, clan, line);
    /* green at 4 and 6: diagonal 6-5-4 -> 5 */
    CHECK(win == 5, "green at 4 and 6 -> winning move 5 (diagonal)");

    board_from_clues(&b, 2, 1, 9, 8);                  /* blue 2,9 -> row 2-9-4 */
    win = board_winning_move(&b, CLAN_BLUE, line);
    CHECK(win == 4, "blue at 2,9 wins at 4 (row 2-9-4)");
    board_from_clues(&b, 1, 2, 4, 8);                  /* blue 1,4 (no shared line); green 2,8 (diagonal 8-5-2) */
    win = board_winning_move(&b, CLAN_BLUE, line);
    CHECK(win == 0, "blue at 1,4 has no winning move");
    int fb = board_fallback_move(&b, CLAN_BLUE);
    CHECK(fb == 5, "fallback blocks green's 8-5-2 diagonal at 5");

    board_from_clues(&b, 5, 3, 0, 10);
    CHECK(b.unresolved_blue == 1 && b.unresolved_green == 1, "0 and 10 are out of range -> unresolved");

    printf("\n%s (%d failures)\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
    return fails ? 1 : 0;
}
