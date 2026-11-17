/* e_section3.ino - Trial Three: the battle of the two Guardians
 *
 * Geometry (rulebook Figure 1, mm):  robot leaves the maze at x~2210 heading +y.
 *   route line  x=2210 : y 2440 -> 3050  (T, turn left = heading 180)
 *   y=3050      : x 2210 -> 2006 (collection crossing) -> 1700 (board edge T)
 *   collection  x=2006 : green piece at y 2700 (left when heading 180), blue at y 3400 (right)
 *   board       transverse lines x 1700 1300 900 500, long lines y 2600 2900 3200 3500
 *   cells       rows x 1500 1100 700 (entrance row first), cols y 2750 3050 3350 (left..right)
 *   exit line   y=3050 : x 500 -> 130 (END)
 */
static const int ROW_X[3] = { 1500, 1100, 700 };
static const int COL_Y[3] = { 2750, 3050, 3350 };

Board board;
static uint8_t occupied[10];       /* inspection result per position */
static uint8_t seenColour[10];     /* colour measured by the gripper sensor during inspection */

/* position -> cell centre */
void posXY(int pos, int *x, int *y) { int r, c; board_row_col(pos, &r, &c); *x = ROW_X[r]; *y = COL_Y[c]; }
int posAt(int row, int col) { return MAGIC[row][col]; }

/* ---- small navigation helpers on the line grid ------------------------- */
/* follow until a junction; tolerate a lost line as "junction" (T end) */
int toJunction(int speed, float maxMM) {
  int r = followLine(speed, maxMM, 0, true);
  if (r == LF_LOST) { driveStraight(LINE_OFFSET_MM, SPEED_SLOW, snapHeading(poseTh)); }
  return r;
}
/* pass n junctions, stopping with the axle on the n-th */
void passJunctions(int n, int speed) {
  for (int i = 0; i < n; i++) {
    toJunction(speed);
    if (i < n - 1) driveStraight(30, SPEED_SLOW, snapHeading(poseTh));   /* clear the junction before looking for the next */
  }
}

/* ---- gripper actions ------------------------------------------------- */
bool pickPiece(int clan) {
  /* standing on the collection crossing (2006,3050) heading 180 (-x) */
  float h = (clan == CLAN_BLUE) ? 90 : -90;           /* blue piece is at +y (robot's right), green at -y */
  turnTo(h);
  jawOpen(); liftDown();
  int r = followLine(SPEED_SLOW, B_COLLECT_LEN - POCKET_OFFSET_MM + 15, 0, false);
  (void)r;
  jawClose();
  int seen = readGripperColour();
  logf("picked piece: gripper sensor says %s", seen == TILE_BLUE ? "BLUE" : seen == TILE_GREEN ? "GREEN" : "unknown");
  bool ok = (seen == TILE_NONE) || (seen == (clan == CLAN_BLUE ? TILE_BLUE : TILE_GREEN));
  if (!ok) logf("WARNING: picked the wrong colour?!");
  liftUp();
  driveStraight(-(B_COLLECT_LEN - POCKET_OFFSET_MM + 15), SPEED_SLOW, h);   /* back to the crossing */
  turnTo(180);
  return ok;
}

/* from a long line, facing the target cell: put the piece down 150 mm away and come back */
void dropPieceAhead(float head) {
  float d = 150 - POCKET_OFFSET_MM;
  driveStraight(d, SPEED_SLOW, head);
  liftDown(); jawOpen();
  driveStraight(-(d + 10), SPEED_SLOW, head);
  driveStraight(10, SPEED_SLOW, head);
}

/* ---- inspection ------------------------------------------------------ */
static int inspLineY, inspLeftCol, inspRightCol, inspDir;   /* dir = -1 : travelling -x from x=1700 ; +1 : +x from 500 */
static float inspX0;
void inspHook() {
  float x = inspX0 + inspDir * travelledMM();
  int l = usPing(PIN_US_L_TRIG, PIN_US_L_ECHO), r = usPing(PIN_US_R_TRIG, PIN_US_R_ECHO);
  for (int i = 0; i < 3; i++) if (fabs(x - ROW_X[i]) < 45) {
    if (l < 130 && inspLeftCol >= 0)  occupied[posAt(i, inspLeftCol)]  = 1;
    if (r < 130 && inspRightCol >= 0) occupied[posAt(i, inspRightCol)] = 1;
  }
}
void inspectSegment(float x0, int dir, int leftCol, int rightCol) {
  inspX0 = x0; inspDir = dir; inspLeftCol = leftCol; inspRightCol = rightCol;
  lfHook = inspHook;
  passJunctions(3, SPEED_SLOW);        /* 1300, 900, 500  (or 900, 1300, 1700 when dir = +1) */
  lfHook = 0;
}

/* Drive the two inner long lines and look sideways for pieces.  Starts and
 * ends on the collection crossing (2006,3050) heading 180. */
void inspectBoard() {
  logf("inspecting the board (clues insufficient)");
  memset(occupied, 0, sizeof occupied); memset(seenColour, 0, sizeof seenColour);
  toJunction();                        /* board edge T at (1700,3050) */
  turnTo(-90); toJunction(SPEED_SLOW, 250);             /* (1700,2900) */
  turnTo(180);
  /* y = 2900 : left (-y) = col 0, right (+y) = col 1 */
  inspectSegment(1700, -1, 0, 1);
  turnTo(90);  passJunctions(1, SPEED_SLOW);            /* up x=500 to (500,3200) : passes the exit T at 3050 ... */
  /* the exit-line T at (500,3050) is seen as a junction too, so pass one more */
  driveStraight(30, SPEED_SLOW, 90); toJunction(SPEED_SLOW, 250);
  turnTo(0);
  /* y = 3200 travelling +x : left (+y) = col 2, right (-y) = col 1 */
  inspectSegment(500, +1, 2, 1);
  turnTo(-90); toJunction(SPEED_SLOW, 250);             /* (1700,3050) entry T */
  turnTo(0);   toJunction(SPEED_NORMAL, 450);           /* back to the collection crossing */
  turnTo(180);
  char s[40]; int n = 0;
  for (int p = 1; p <= 9; p++) if (occupied[p]) n += snprintf(s + n, sizeof s - n, "%d ", p);
  logf("occupied positions: %s", s);
}

/* resolve unknown pieces: clue colours that could not be placed are assigned to
 * occupied positions that the clues did not explain.  If both colours are
 * unresolved and we must choose, take the gripper reading if we have one,
 * otherwise assume the piece nearer the entrance belongs to the opponent
 * (harmless: it only changes a losing fallback move). */
void resolveBoard() {
  int unknown[9], nu = 0;
  for (int p = 1; p <= 9; p++) if (occupied[p] && board.piece[p] == 0) unknown[nu++] = p;
  for (int p = 1; p <= 9; p++) if (board.piece[p] && !occupied[p]) { logf("position %d expected a piece but none seen - dropping", p); board.piece[p] = 0; }
  int nb = board.unresolved_blue, ng = board.unresolved_green;
  for (int i = 0; i < nu; i++) {
    int p = unknown[i];
    if (seenColour[p] == TILE_BLUE && nb) { board.piece[p] = CLAN_BLUE; nb--; }
    else if (seenColour[p] == TILE_GREEN && ng) { board.piece[p] = CLAN_GREEN; ng--; }
    else if (nb && !ng) { board.piece[p] = CLAN_BLUE; nb--; }
    else if (ng && !nb) { board.piece[p] = CLAN_GREEN; ng--; }
    else if (nb && ng) { board.piece[p] = (myClan == CLAN_BLUE) ? CLAN_GREEN : CLAN_BLUE; if (board.piece[p] == CLAN_BLUE) nb--; else ng--; }
  }
}

/* ---- placing --------------------------------------------------------- */
/* Starting on the board-edge T (1700,3050) heading 180. Places at `pos`, then
 * continues to the END zone. */
void placeAndFinish(int pos) {
  int row, col; board_row_col(pos, &row, &col);
  int tx = ROW_X[row];
  /* choose the long line: col 0 -> y 2900 facing -y ; col 1 -> y 2900 facing +y ; col 2 -> y 3200 facing +y */
  int lineY = (col == 2) ? 3200 : 2900;
  float faceH = (col == 0) ? -90 : 90;
  turnTo(lineY < 3050 ? -90 : 90);
  toJunction(SPEED_SLOW, 250);                          /* onto the long line */
  turnTo(180);
  int cross = (1700 - tx) / 400;                        /* junctions to pass before the cell: 0,1,2 */
  passJunctions(cross, SPEED_NORMAL);
  if (cross) driveStraight(30, SPEED_SLOW, 180);
  followLine(SPEED_SLOW, (cross ? 170 : 200) - 0, 0, false);   /* to x = tx (200 mm past the last junction) */
  logf("placing at position %d (%d,%d)", pos, tx, COL_Y[col]);
  turnTo(faceH);
  dropPieceAhead(faceH);
  turnTo(180);
  passJunctions(3 - cross, SPEED_NORMAL);               /* remaining junctions to the x=500 edge line */
  /* to the exit line y=3050 */
  turnTo(lineY < 3050 ? 90 : -90);
  toJunction(SPEED_SLOW, 250);                          /* T at (500,3050) */
  turnTo(180);
  followLine(SPEED_NORMAL, 420, 0, false);              /* END zone */
  stopMotors();
  logf("END reached");
}

/* ---- main ------------------------------------------------------------ */
void runSection3(bool restartedHere) {
  logf("===== SECTION 3: BATTLE OF THE TWO GUARDIANS =====");
  unsigned long t0 = millis();
  myClan = board_from_clues(&board, clueBlue, clueGreen, clueRight, clueLeft);
  digitalWrite(PIN_LED_BLUE, myClan == CLAN_BLUE); digitalWrite(PIN_LED_GREEN, myClan == CLAN_GREEN);
  logf("clues B=%d G=%d R=%d L=%d -> %s ; unresolved blue %d green %d", clueBlue, clueGreen, clueRight, clueLeft,
       myClan == CLAN_BLUE ? "BLUE (cylinder)" : "GREEN (cube)", board.unresolved_blue, board.unresolved_green);

  /* 1. from the maze exit door to the collection crossing */
  if (restartedHere) poseTh = 90;
  followLine(SPEED_NORMAL, 700, 0, true);               /* route line up to the T at (2210,3050) */
  driveStraight(0, 0, 90);
  turnTo(180);
  toJunction();                                         /* collection crossing (2006,3050) */

  /* 2. inspect if needed (before picking, so the gripper sensor is free) */
  bool needInspect = (board.unresolved_blue + board.unresolved_green) > 0;
  if (needInspect) { inspectBoard(); resolveBoard(); }

  /* 3. decide */
  uint8_t line[3];
  int win = board_winning_move(&board, myClan, line);
  if (win) logf("winning move: %d (line %d-%d-%d)", win, line[0], line[1], line[2]);
  else { win = board_fallback_move(&board, myClan); logf("no winning move - fallback position %d", win); }
  char b[32]; for (int r = 0; r < 3; r++) { for (int c = 0; c < 3; c++) b[r * 4 + c] = ".BG"[board.piece[MAGIC[r][c]]]; b[r * 4 + 3] = (r < 2) ? '/' : 0; }
  logf("board (entrance row first, left->right): %s", b);

  /* 4. pick, 5. place and finish */
  pickPiece(myClan);
  toJunction();                                         /* board edge T (1700,3050) */
  placeAndFinish(win);
  logf("S3 done t=%.1fs", (millis() - t0) / 1000.0);
}
