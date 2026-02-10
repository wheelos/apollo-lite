import curses
import threading
from datetime import datetime


class UIHandler:
    """
    Thread-safe abstraction for Curses UI.
    Splits screen into Header, Log Area, and Status Footer.
    """

    # Color Pairs
    PAIR_DEFAULT = 1
    PAIR_GREEN = 2
    PAIR_RED = 3
    PAIR_YELLOW = 4
    PAIR_CYAN = 5

    COLOR_MAP = {
        "DEFAULT": PAIR_DEFAULT,
        "GREEN": PAIR_GREEN,
        "RED": PAIR_RED,
        "YELLOW": PAIR_YELLOW,
        "CYAN": PAIR_CYAN,
    }

    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.lock = threading.Lock()
        self._setup_curses()

    def _setup_curses(self):
        curses.curs_set(0)  # Hide cursor
        self.stdscr.nodelay(True)  # Non-blocking input
        curses.start_color()
        curses.use_default_colors()

        # Init pairs
        curses.init_pair(self.PAIR_DEFAULT, curses.COLOR_WHITE, -1)
        curses.init_pair(self.PAIR_GREEN, curses.COLOR_GREEN, -1)
        curses.init_pair(self.PAIR_RED, curses.COLOR_RED, -1)
        curses.init_pair(self.PAIR_YELLOW, curses.COLOR_YELLOW, -1)
        curses.init_pair(self.PAIR_CYAN, curses.COLOR_CYAN, -1)

    def clear(self):
        with self.lock:
            self.stdscr.clear()
            self.stdscr.refresh()

    def get_input(self):
        """Non-blocking input retrieval"""
        try:
            return self.stdscr.getch()
        except curses.error:
            return -1

    def wait_for_enter(self):
        """Blocking wait for Enter key"""
        self.stdscr.nodelay(False)
        while True:
            key = self.stdscr.getch()
            if key == 10 or key == 13:  # Enter
                break
        self.stdscr.nodelay(True)

    def draw_header(self, title, level):
        with self.lock:
            h, w = self.stdscr.getmaxyx()
            header_text = f"{title} | Level: {level}"
            self.stdscr.addstr(
                0,
                0,
                header_text[: w - 1],
                curses.color_pair(self.PAIR_CYAN) | curses.A_BOLD,
            )
            self.stdscr.addstr(
                1, 0, "-" * (w - 1), curses.color_pair(self.PAIR_DEFAULT)
            )

            # Safety instruction always visible
            safety_msg = "🚨 PRESS [SPACEBAR] FOR EMERGENCY STOP 🚨"
            self.stdscr.addstr(
                h - 1,
                0,
                safety_msg[: w - 1],
                curses.color_pair(self.PAIR_RED) | curses.A_REVERSE,
            )
            self.stdscr.refresh()

    def log(self, message: str, color: str = "DEFAULT"):
        """
        Scrollable log output.
        """
        with self.lock:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            full_msg = f"[{timestamp}] {message}"

            h, w = self.stdscr.getmaxyx()
            # Log area is between header (lines 0-1) and footer (line h-1)
            # Simplistic approach: just append and let curses scroll or handle manually
            # For robustness, we usually just write to the line above footer and scroll up

            y, x = self.stdscr.getyx()

            # If at bottom of log area, clear and reset (simple ring buffer visual)
            if y >= h - 2:
                self.stdscr.move(2, 0)
                self.stdscr.clrtobot()
                # Redraw footer
                safety_msg = "🚨 PRESS [SPACEBAR] FOR EMERGENCY STOP 🚨"
                self.stdscr.addstr(
                    h - 1,
                    0,
                    safety_msg[: w - 1],
                    curses.color_pair(self.PAIR_RED) | curses.A_REVERSE,
                )
                self.stdscr.move(2, 0)

            pair = self.COLOR_MAP.get(color.upper(), self.PAIR_DEFAULT)
            self.stdscr.addstr(full_msg + "\n", curses.color_pair(pair))
            self.stdscr.refresh()
