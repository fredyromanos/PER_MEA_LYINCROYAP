"""
Test bootstrap for the IHM ground-station pure functions.

Why the fakes: the real FastAPI/pydantic/pymongo/pyserial stack is not runnable
in this environment (venv built for py3.12, host is py3.14; broken starlette; no
network to reinstall). But the functions we care about — the LoRa wire-protocol
builders/parsers and JSON decoders — are pure stdlib logic. So we inject minimal
fakes for the heavy imports into sys.modules and then load the REAL module source
with importlib. The function bodies under test are the project's real code; only
their (unused-by-these-functions) dependencies are faked.

If the real stack is ever installed, these same tests still pass unchanged because
the fakes are only registered when the real import is absent.
"""
import importlib.util
import os
import sys
import types

import pytest

APP_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "app"))


# ── minimal fakes for heavy third-party / project deps ───────────────────────
def _install_fakes():
    # pyserial
    sys.modules.setdefault("serial", types.ModuleType("serial"))

    # pydantic: BaseModel stores kwargs as attributes; Field returns its default.
    if "pydantic" not in sys.modules:
        pyd = types.ModuleType("pydantic")

        class BaseModel:
            def __init__(self, **kw):
                for k, v in kw.items():
                    setattr(self, k, v)

        def Field(default=None, **kw):
            return default

        pyd.BaseModel = BaseModel
        pyd.Field = Field
        sys.modules["pydantic"] = pyd

    # fastapi + fastapi.responses
    if "fastapi" not in sys.modules:
        fa = types.ModuleType("fastapi")

        class APIRouter:
            def _dec(self, *a, **k):
                def wrap(fn):
                    return fn
                return wrap
            get = post = put = delete = _dec

        fa.APIRouter = APIRouter
        sys.modules["fastapi"] = fa

        far = types.ModuleType("fastapi.responses")

        class JSONResponse:
            def __init__(self, content, status_code=200):
                self.content = content
                self.status_code = status_code

        far.JSONResponse = JSONResponse
        fa.responses = far
        sys.modules["fastapi.responses"] = far

    # project config (heavy: dotenv + serial port probing) → faked
    if "config" not in sys.modules:
        cfg = types.ModuleType("config")
        cfg.SERVER_PORT = 5000
        cfg.SERIAL_PORT = "/tmp/ttyV1"
        cfg.BAUD_RATE = 115200
        cfg.SIMULATION = True

        def _noop(*a, **k):
            return None
        cfg.find_transceiver_port = _noop
        cfg.resolve_serial_port = lambda: ("/tmp/ttyV1", "fake")
        sys.modules["config"] = cfg

    # utils.db (MongoDB) → faked; push/sync_client/get_pending_message are no-ops.
    if "utils" not in sys.modules:
        utils = types.ModuleType("utils")
        utils.__path__ = []  # mark as package
        sys.modules["utils"] = utils
    if "utils.db" not in sys.modules:
        db = types.ModuleType("utils.db")

        def push(*a, **k):
            return None

        def sync_client(*a, **k):
            return object()

        def get_pending_message(*a, **k):
            return None

        db.push = push
        db.sync_client = sync_client
        db.get_pending_message = get_pending_message
        sys.modules["utils.db"] = db
        sys.modules["utils"].db = db


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


_install_fakes()
if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)

# Load the REAL project source (function bodies under test) under the fakes.
_messages = _load(os.path.join(APP_DIR, "routes", "messages.py"), "routes.messages")
_serial_link = _load(os.path.join(APP_DIR, "serial_link.py"), "serial_link")


@pytest.fixture
def messages():
    return _messages


@pytest.fixture
def serial_link():
    return _serial_link
