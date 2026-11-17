/* d_section2.ino - Trial Two: the labyrinth
 * Frontier exploration of the 9 x 4 maze with ultrasonic wall sensing at each cell
 * centre, floor colour sensor for the tiles in dead ends, then shortest-path turn
 * counting (game_logic.h) and navigation to the exit cell. */

static int mc = MZ_ENTRY_C, mr = MZ_ENTRY_R, mh = H_N;   /* cell and heading index */
static const float HEAD_DEG[4] = { 90, 0, -90, 180 };     /* H_N, H_E, H_S, H_W  -> world degrees */
static uint8_t tileVote[MZ_COLS][MZ_ROWS];                /* strongest colour seen while entering the cell */

/* sense walls around the current cell from its centre */
void mzSenseCell() {
  readUltrasonics(true);
  int f = usF + (int)US_F_OFFSET_MM, l = usL + (int)US_SIDE_OFFSET_MM, r = usR + (int)US_SIDE_OFFSET_MM, b = usB + (int)US_F_OFFSET_MM;
  mz_set_wall(&maze, mc, mr, mh,           f < WALL_SEEN_MM ? W_WALL : W_OPEN);
  mz_set_wall(&maze, mc, mr, (mh + 3) & 3, l < WALL_SEEN_MM ? W_WALL : W_OPEN);   /* left  = heading - 1 */
  mz_set_wall(&maze, mc, mr, (mh + 1) & 3, r < WALL_SEEN_MM ? W_WALL : W_OPEN);   /* right = heading + 1 */
  if (maze.wall[mc][mr][(mh + 2) & 3] == W_UNKNOWN) mz_set_wall(&maze, mc, mr, (mh + 2) & 3, b < WALL_SEEN_MM ? W_WALL : W_OPEN);
  maze.visited[mc][mr] = 1;
  logf("cell (%d,%d) h=%c F%d L%d R%d B%d walls:%c%c%c%c", mc, mr, "NESW"[mh], f, l, r, b,
       maze.wall[mc][mr][0] == W_WALL ? 'N' : '.', maze.wall[mc][mr][1] == W_WALL ? 'E' : '.',
       maze.wall[mc][mr][2] == W_WALL ? 'S' : '.', maze.wall[mc][mr][3] == W_WALL ? 'W' : '.');
  if (mz_is_dead_end(&maze, mc, mr) && !maze.tile[mc][mr]) {
    /* dead end: confirm the tile with a few stationary samples (sensor is ~50 mm ahead of the axle) */
    int votes[3] = { 0, 0, 0 };
    for (int i = 0; i < 6; i++) { int t = readFloorColour(); if (t) votes[t]++; }
    int t = tileVote[mc][mr];
    if (votes[TILE_BLUE] >= 3) t = TILE_BLUE; else if (votes[TILE_GREEN] >= 3) t = TILE_GREEN;
    maze.tile[mc][mr] = t;
    if (t) { logf("TILE %s at (%d,%d)", t == TILE_BLUE ? "BLUE" : "GREEN", mc, mr); beep(1, 40); }
  }
}

/* re-centre using the front wall if there is one (fixes encoder drift along the cell) */
void mzSquareUp() {
  usF = usMedian(PIN_US_F_TRIG, PIN_US_F_ECHO);
  if (usF < WALL_SEEN_MM - 20) {
    float err = usF - FRONT_STOP_MM;                 /* +: too far from the wall */
    if (fabs(err) > 8 && fabs(err) < 80) driveStraight(err, SPEED_SLOW, HEAD_DEG[mh]);
  }
}

/* move one cell in direction h (turning first if needed) */
void mzMove(int h) {
  if (h != mh) { turnTo(HEAD_DEG[h]); mh = h; }
  int tile = TILE_NONE;
  driveStraight(MZ_CELL_MM, SPEED_NORMAL, HEAD_DEG[mh], true, FRONT_STOP_MM, &tile);
  mc += MZ_DX[mh]; mr += MZ_DY[mh];
  if (tile) tileVote[mc][mr] = tile;
  mzSquareUp();
}

void mzEnter() {
  /* standing in front of the entrance door facing +y: drive to the first cell centre.
   * Door wall line is at y = 1330 (Figure 1); cell centre = 1330 + 125. */
  float d = (1330 + MZ_CELL_MM / 2) - poseY;
  if (d < 60 || d > 600) d = 200;
  int tile = TILE_NONE;
  driveStraight(d, SPEED_NORMAL, 90, true, FRONT_STOP_MM, &tile);
  mc = MZ_ENTRY_C; mr = MZ_ENTRY_R; mh = H_N;
  if (tile) tileVote[mc][mr] = tile;
  mzSquareUp();
}

void mzFollowPath(const MazePath *p, bool stopAtFrontier) {
  for (int i = 0; i < p->len; i++) {
    int h = p->h[i];
    if (maze.wall[mc][mr][h] == W_WALL) { logf("path blocked by a newly seen wall"); return; }
    mzMove(h);
    mzSenseCell();
    if (stopAtFrontier && i < p->len - 1) {
      /* new info may have made the rest of the plan pointless; re-plan after each new cell */
      int nc = mc + MZ_DX[p->h[i + 1]], nr = mr + MZ_DY[p->h[i + 1]];
      if (maze.wall[mc][mr][p->h[i + 1]] == W_WALL || (mz_in(nc, nr) && maze.visited[nc][nr] && i + 1 < p->len - 1)) return;
    }
  }
}

void runSection2() {
  logf("===== SECTION 2: LABYRINTH =====");
  unsigned long t0 = millis();
  mz_init(&maze); memset(tileVote, 0, sizeof tileVote);
  mzEnter();
  mzSenseCell();
  MazePath path;
  int gc, gr, guard = 0;
  while (mz_nearest_frontier(&maze, mc, mr, mh, &gc, &gr)) {
    if (++guard > 120 || millis() - t0 > 240000) { logf("S2: exploration guard hit"); break; }
    if (!mz_plan(&maze, mc, mr, mh, gc, gr, 1, &path)) { maze.visited[gc][gr] = 1; continue; }
    mzFollowPath(&path, true);
  }
  int nb, ng; mz_tile_counts(&maze, &nb, &ng);
  if (!mz_plan(&maze, MZ_ENTRY_C, MZ_ENTRY_R, H_N, MZ_EXIT_C, MZ_EXIT_R, 0, &path))
    mz_plan(&maze, MZ_ENTRY_C, MZ_ENTRY_R, H_N, MZ_EXIT_C, MZ_EXIT_R, 1, &path);
  clueBlue = nb; clueGreen = ng; clueRight = path.right_turns; clueLeft = path.left_turns;
  myClan = (nb > ng) ? CLAN_BLUE : CLAN_GREEN;
  digitalWrite(PIN_LED_BLUE, myClan == CLAN_BLUE); digitalWrite(PIN_LED_GREEN, myClan == CLAN_GREEN);
  logf("explored in %.1fs: blue=%d green=%d  shortest path %d cells: R=%d L=%d -> %s",
       (millis() - t0) / 1000.0, nb, ng, path.len, clueRight, clueLeft, myClan == CLAN_BLUE ? "BLUE SENTINELS" : "GREEN KEEPERS");
  saveClues();
  /* go to the exit cell and out through the door */
  if (mz_plan(&maze, mc, mr, mh, MZ_EXIT_C, MZ_EXIT_R, 0, &path) || mz_plan(&maze, mc, mr, mh, MZ_EXIT_C, MZ_EXIT_R, 1, &path))
    mzFollowPath(&path, false);
  if (mh != MZ_EXIT_HEAD) { turnTo(HEAD_DEG[MZ_EXIT_HEAD]); mh = MZ_EXIT_HEAD; }
  driveStraight(MZ_CELL_MM, SPEED_NORMAL, HEAD_DEG[mh], true, 0);      /* through the exit door */
  logf("S2 done t=%.1fs", (millis() - t0) / 1000.0);
}
