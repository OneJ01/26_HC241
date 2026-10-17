"""GUI constructor regression tests without requiring a real Qt display or ROS2."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import textwrap


SOURCE = Path(__file__).resolve().parents[1] / "mmr_gui" / "main_gui.py"
TEXT = SOURCE.read_text(encoding="utf-8")


def test_runtime_caches_are_initialized_before_hud_construction():
    hud_index = TEXT.index("self.hud = self._build_hud_panel()")
    required_initializers = (
        "self._gui_align_aim_pressed: bool = False",
        "self._gui_fire_pressed: bool = False",
        'self._last_engagement_state: str = ""',
        'self._last_operator_style_key: str = ""',
        'self._last_fire_status_key: str = ""',
        "self._engagement_status_stamp: float = 0.0",
    )
    for initializer in required_initializers:
        assert TEXT.index(initializer) < hud_index, initializer


def test_publishers_have_safe_defaults_before_ros_setup():
    setup_index = TEXT.index("self._setup_ros()")
    for initializer in (
        "self._pub_operator_input = None",
        "self._pub_turret_home_request = None",
        "self._pub_emergency_stop = None",
        "self._pub_target_point = None",
    ):
        assert TEXT.index(initializer) < setup_index, initializer


def test_main_window_constructor_smoke_with_stubbed_qt_and_without_ros():
    """Catch AttributeError caused by widget-build/attribute-init ordering."""
    script = textwrap.dedent(
        f"""
        import sys, types, importlib.util

        class Any:
            def __init__(self, *a, **kw):
                self._text = ''
                self._style = ''
                self._tooltip = ''
            def __call__(self, *a, **kw): return Any()
            def __getattr__(self, name):
                if name in {{'clicked','pressed','released','timeout','loadFinished',
                             'map_clicked','map_right_clicked','dem_clicked',
                             'test_target_right_clicked','triggered'}}:
                    return Signal()
                return Any()
            def __or__(self, other): return self
            def __ror__(self, other): return self
            def __and__(self, other): return self
            def __rand__(self, other): return self
            def __bool__(self): return False
            def __int__(self): return 0
            def __float__(self): return 0.0
            def __iter__(self): return iter(())
            def __len__(self): return 0
            def connect(self, *a, **kw): return None
            def emit(self, *a, **kw): return None
            def setText(self, value): self._text = str(value)
            def text(self): return self._text
            def setStyleSheet(self, value): self._style = str(value)
            def styleSheet(self): return self._style
            def setToolTip(self, value): self._tooltip = str(value)
            def toolTip(self): return self._tooltip
            def width(self): return 1280
            def height(self): return 800
            def rect(self): return Any()
            def page(self): return self
            def menuBar(self): return self
            def addMenu(self, *a, **kw): return self
            def addAction(self, *a, **kw): return self

        class Signal(Any):
            def __get__(self, instance, owner): return self
        class Widget(Any): pass
        class QMainWindow(Widget): pass
        class QWidget(Widget): pass
        class QFrame(Widget): pass
        class QObject(Any): pass
        class QTimer(Any):
            @staticmethod
            def singleShot(*a, **kw): return None
        class QApplication(Any):
            def exec(self): return 0
        class QPixmap(Any):
            @staticmethod
            def fromImage(*a, **kw): return QPixmap()
        class Enum(Any): pass

        Qt = Enum()
        QFont = type('QFont', (Any,), {{'Weight': Enum()}})
        QPainter = type('QPainter', (Any,), {{'RenderHint': Enum()}})
        QImage = type('QImage', (Any,), {{'Format': Enum()}})
        QSizePolicy = type('QSizePolicy', (Any,), {{'Policy': Enum()}})

        core = types.ModuleType('PyQt6.QtCore')
        for key, value in dict(Qt=Qt, QTimer=QTimer,
            pyqtSignal=lambda *a, **k: Signal(), QPointF=Any,
            QObject=QObject, pyqtSlot=lambda *a, **k: (lambda f: f)).items():
            setattr(core, key, value)
        gui = types.ModuleType('PyQt6.QtGui')
        for key, value in dict(QShortcut=Any, QKeySequence=Any, QFont=QFont,
            QAction=Any, QPainter=QPainter, QPen=Any, QBrush=Any,
            QPolygonF=Any, QColor=Any, QImage=QImage, QPixmap=QPixmap).items():
            setattr(gui, key, value)
        widgets = types.ModuleType('PyQt6.QtWidgets')
        for key, value in dict(QApplication=QApplication, QMainWindow=QMainWindow,
            QWidget=QWidget, QVBoxLayout=Any, QHBoxLayout=Any, QLabel=Widget,
            QStackedWidget=Widget, QTextEdit=Widget, QFrame=QFrame,
            QSplitter=Widget, QSizePolicy=QSizePolicy, QPushButton=Widget).items():
            setattr(widgets, key, value)
        webengine = types.ModuleType('PyQt6.QtWebEngineWidgets')
        webengine.QWebEngineView = Widget
        webchannel = types.ModuleType('PyQt6.QtWebChannel')
        webchannel.QWebChannel = Widget
        sys.modules.update({{
            'PyQt6': types.ModuleType('PyQt6'),
            'PyQt6.QtCore': core,
            'PyQt6.QtGui': gui,
            'PyQt6.QtWidgets': widgets,
            'PyQt6.QtWebEngineWidgets': webengine,
            'PyQt6.QtWebChannel': webchannel,
        }})

        spec = importlib.util.spec_from_file_location('gui_under_test', {str(SOURCE)!r})
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.MainWindow()
        print('GUI_STARTUP_SMOKE_OK')
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "GUI_STARTUP_SMOKE_OK" in result.stdout
