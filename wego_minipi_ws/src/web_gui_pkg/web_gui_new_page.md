해결되어서 다행입니다! 🚀
앞으로 새로운 페이지를 추가하고 싶으실 때, **딱 4단계**만 기억하시면 됩니다.

이번에 겪으셨던 시행착오를 바탕으로 **가장 중요한 포인트**를 짚어드릴게요.

---

### 1단계: UI 파일 생성 (`page_새기능.py`)
`src/web_gui_pkg/src/web_gui_pkg/` 폴더 안에 새 파이썬 파일을 만듭니다.

```python
# src/web_gui_pkg/src/web_gui_pkg/page_new.py

from nicegui import ui
from .core_logic import NiceGUIRos_instance # 로직 필요 시 임포트

@ui.page("/new_page")  # <--- 접속할 주소 설정
def new_page_func():
    # 헤더 (기존 코드 복사해서 쓰면 편함)
    with ui.header().classes("bg-slate-900"):
        ui.label("새로운 페이지").classes("text-white font-bold")
        ui.button("메인으로", on_click=lambda: ui.navigate.to("/"))

    # 내용
    ui.label("여기에 기능을 추가하세요!").classes("text-2xl mt-10")
```

---

### 2단계: 로직 추가 (선택 사항)
`src/web_gui_pkg/src/web_gui_pkg/core_logic.py`의 `NiceGUIRos` 클래스 안에 필요한 변수나 함수를 추가합니다.

```python
# core_logic.py

class NiceGUIRos:
    def __init__(self):
        # ...
        self.new_variable = 0 # 새 상태 변수

    def do_something_new(self):
        print("새로운 기능 실행!")
```

---

### 3단계: 페이지 등록 (★ 가장 중요 ★)
이번에 에러가 났던 원인입니다. **두 파일 모두에 `import`를 해야 합니다.**

**1. `src/web_gui_pkg/src/web_gui_pkg/main_app.py`**
```python
# ...
from .page_launch import main_page
from .page_cam_setting import cam_setting_page
from .page_new import new_page_func  # <--- 추가
```

**2. `src/web_gui_pkg/scripts/run_webgui.py` (실행 파일)**
```python
# ...
import web_gui_pkg.page_launch
import web_gui_pkg.page_cam_setting
import web_gui_pkg.page_new  # <--- 여기도 반드시 추가! (안 하면 404 에러)
```

---

### 4단계: 이동 버튼 만들기
메인 페이지(`page_launch.py`)나 헤더에 이동 버튼을 만듭니다.

```python
# page_launch.py 등

ui.button("새 기능 가기", on_click=lambda: ui.navigate.to("/new_page"))
```

*(만약 이번처럼 `roslaunch` 리스트에 가짜로 띄우고 싶다면 `core_logic.py`의 `find_launch_files` 함수에 강제로 딕셔너리를 추가해주면 됩니다.)*

이 순서대로만 진행하시면 문제없이 확장이 가능합니다! 고생하셨습니다.