# Установка среды с нуля — полное руководство

Этот файл описывает **всё**, что нужно установить на чистой Windows-машине, чтобы запустить проект IsaacPilot: WSL2 (Linux внутри Windows), VS Code, Python-окружение для Isaac Sim, Jupyter kernel для notebook'ов и ArduPilot SITL.

Порядок в файле = порядок установки. Каждая команда объяснена: что делает, что будет, если поменять параметр.

---

## 1. WSL2 (Linux внутри Windows)

### Зачем он нужен
ArduPilot SITL (симулятор полётного контроллера) официально собирается и работает только под Linux. Isaac Sim при этом ставится на Windows (нужна видеокарта NVIDIA с драйверами напрямую в хост-системе). WSL2 — это способ запустить настоящее ядро Linux (Ubuntu) внутри Windows без второго компьютера и без виртуалки в привычном понимании (VirtualBox/VMware) — Microsoft встроил лёгкую виртуализацию прямо в Windows.

### Требования
- Windows 10 версии 2004+ (сборка 19041+) или Windows 11
- В BIOS/UEFI должна быть включена виртуализация (Intel VT-x / AMD-V) — на большинстве современных ПК включена по умолчанию
- Проверить версию Windows: `Win+R` → `winver`

### Установка
Открой **PowerShell от имени администратора** (правый клик на Пуск → «Терминал (администратор)» или «Windows PowerShell (администратор)») и выполни:

```powershell
wsl --install
```

**Что делает эта команда:**
- Включает нужные компоненты Windows (Virtual Machine Platform, Windows Subsystem for Linux) — это фичи самой Windows, до этого выключенные
- Скачивает и ставит **Ubuntu** (дистрибутив по умолчанию) — конкретно последнюю LTS-версию, доступную в Microsoft Store
- Устанавливает WSL версии 2 (в отличие от WSL1, WSL2 — это настоящее linux-ядро, а не транслятор системных вызовов; SITL и сеть работают только на WSL2)

Если хочешь конкретную версию Ubuntu (например, если проект тестировался на 22.04), укажи явно:
```powershell
wsl --install -d Ubuntu-22.04
```
Список всех доступных дистрибутивов: `wsl --list --online`.

### Перезагрузка
После `wsl --install` **обязательно перезагрузи компьютер** — команда включает системные компоненты Windows, которые применяются только после рестарта.

### Первый запуск
После перезагрузки Ubuntu запустится сама (или найди её в меню Пуск → Ubuntu). Первый запуск попросит:
1. Придумать **имя пользователя Linux** (может отличаться от имени пользователя Windows, латиницей, без пробелов)
2. Придумать **пароль** — вводится вслепую (символы не отображаются даже точками), это нормально для Linux-терминалов

Этот пользователь становится администратором (sudo) внутри Ubuntu.

### Проверка, что всё встало
В обычном PowerShell (не в Ubuntu):
```powershell
wsl --list --verbose
```
Должно показать что-то вроде:
```
  NAME      STATE           VERSION
* Ubuntu    Running         2
```
Колонка **VERSION должна быть 2**, не 1 — иначе SITL и сеть работать не будут. Если стоит 1: `wsl --set-version Ubuntu 2`.

### Обновление системы
Внутри Ubuntu (открой её из Пуск, или в PowerShell набери `wsl`):
```bash
sudo apt update && sudo apt upgrade -y
```
`apt update` — обновляет список доступных пакетов (не сами пакеты); `apt upgrade -y` — ставит свежие версии уже установленных пакетов, `-y` значит «отвечать да на все вопросы автоматически». Это стандартный первый шаг на любой свежей Ubuntu.

---

## 2. Visual Studio Code

### Скачивание
Официальный сайт: **code.visualstudio.com** → кнопка «Download for Windows». Скачается `.exe`-установщик (System Installer, ставится на весь компьютер — рекомендуется, в отличие от User Installer).

### Установка
Запусти скачанный `.exe`. Во время установки на экране «Select Additional Tasks» рекомендую отметить галочками:
- **Add "Open with Code" action to Windows Explorer file context menu** — правый клик на папке → «Open with Code», удобно
- **Add to PATH** (обычно уже отмечено) — без этого команда `code` не будет работать в терминале

Остальные шаги — «Next» по умолчанию.

### Нужные расширения
Открой VS Code → значок квадратиков слева (Extensions, `Ctrl+Shift+X`) → в поиске введи и поставь (кнопка Install):

| Расширение | ID | Зачем |
|---|---|---|
| **Python** | `ms-python.python` | Подсветка синтаксиса, автодополнение, запуск .py файлов |
| **Jupyter** | `ms-toolsai.jupyter` | Открытие и запуск `.ipynb` notebook'ов прямо в VS Code (наши `drone_control_ardupilot.ipynb` и `flight_missions.ipynb`) |
| **WSL** | `ms-vscode-remote.remote-wsl` | Позволяет открыть VS Code «внутри» Ubuntu (папку `/home/...` или `/mnt/c/...` с точки зрения Linux) — пригодится при правке файлов ArduPilot |

После установки — перезапусти VS Code (предложит сам кнопкой «Reload»).

---

## 3. Python-окружение для Isaac Sim (`isaac_env`)

Isaac Sim 6.0 ставится не как отдельная программа с инсталлятором, а как **обычный Python-пакет через pip**, внутри изолированного виртуального окружения (venv). Так разработчик может держать несколько версий Isaac Sim / Python рядом без конфликтов.

### 3.1 Включить длинные пути Windows (LongPaths)
Isaac Sim распаковывает тысячи файлов с очень длинными путями (глубоко вложенные папки экстеншенов) — стандартный лимит Windows в 260 символов на путь этого не выдерживает, установка падает с непонятными ошибками. Это нужно включить **один раз на систему, ДО установки Isaac Sim**.

PowerShell **от администратора**:
```powershell
Set-ItemProperty -Path 'HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem' -Name 'LongPathsEnabled' -Value 1
```
**Что делает:** меняет значение реестра Windows `LongPathsEnabled` на 1 (включено). Реестр — это системная база настроек Windows; `HKLM:\...` означает «относится ко всей машине», не к одному пользователю. Перезагрузка после этого не обязательна, но безопаснее перезайти в систему.

### 3.2 Установить Python 3.12
Isaac Sim 6.0.1.0 рассчитан на Python 3.12 (в проекте использовалась 3.12.7). Скачать с **python.org/downloads** → раздел Windows → «Windows installer (64-bit)» для версии 3.12.x.

При установке **обязательно отметь галочку «Add python.exe to PATH»** на первом экране инсталлятора — без неё команда `python` в терминале не найдётся.

Проверка после установки (открой новый PowerShell, старые окна не подхватят PATH):
```powershell
python --version
```
Должно вывести `Python 3.12.7` (или близкую 3.12.x).

### 3.3 Создать виртуальное окружение
В PowerShell перейди в папку проекта и создай venv с именем `isaac_env`:
```powershell
cd C:\VSCODE\ISAAC
python -m venv isaac_env
```
**Что делает:** `python -m venv <имя>` создаёт папку `isaac_env` — внутри неё копия интерпретатора Python и пустое место под библиотеки, изолированное от системного Python. Так пакеты Isaac Sim не смешаются с другими Python-проектами на компьютере.

Активировать окружение (нужно делать в каждом новом терминале перед работой с проектом):
```powershell
.\isaac_env\Scripts\activate
```
После активации в начале строки терминала появится `(isaac_env)` — это знак, что все `pip install` и `python` дальше идут внутрь этого изолированного окружения, а не в системный Python.

### 3.4 Установить Isaac Sim
Внутри активированного `isaac_env`:
```powershell
pip install isaacsim[all,extscache]==6.0.1.0 --extra-index-url https://pypi.nvidia.com
```
**Что делает:**
- `isaacsim[all,extscache]` — ставит основной пакет Isaac Sim плюс **все** опциональные экстеншены (`all`) и кэш экстеншенов (`extscache`, чтобы не тянуть их по сети при первом запуске)
- `==6.0.1.0` — фиксирует точную версию. **Не убирать** — на других версиях API `omni.physx`/`isaacsim.code_editor` может отличаться, весь код моста писался и тестировался под 6.0.1.0
- `--extra-index-url` — Isaac Sim пакеты лежат не на обычном PyPI, а на отдельном сервере NVIDIA, эта опция говорит pip искать также и там

Установка тяжёлая (несколько гигабайт) и может занять 10-20+ минут в зависимости от интернета.

### 3.5 Первый запуск Isaac Sim
Из активированного окружения:
```powershell
isaacsim
```
Первый запуск дольше обычного — Isaac Sim компилирует шейдеры и разворачивает кэш экстеншенов. Дальше открывается основное окно (Viewport, Stage, Property и т.д. — описаны в `interface_guide.md`).

### 3.6 Включить нужный экстеншен
Внутри Isaac Sim: меню **Window → Extensions** → в поиске найди `isaacsim.code_editor.python_server` → включи галочкой (toggle). Это открывает TCP-порт **8226**, через который notebook (`bridge_up()`) шлёт команды прямо в работающий Isaac Sim.

**Важно:** экстеншен `isaacsim.code_editor.jupyter` держать **выключенным** — в версии 6.0.1.0 он конфликтует и ломает сессию. Пользуемся только `python_server`.

---

## 4. Jupyter kernel для notebook'ов

Notebook'и проекта (`drone_control_ardupilot.ipynb`, `flight_missions.ipynb`) должны выполняться именно тем Python-интерпретатором, где стоит Isaac Sim и все библиотеки моста — то есть внутри `isaac_env`. Jupyter называет такой интерпретатор **kernel** (ядро). Чтобы VS Code увидел `isaac_env` как отдельный kernel в списке, его нужно **зарегистрировать**.

### 4.1 Поставить ipykernel в окружение
В активированном `isaac_env` (терминал должен показывать `(isaac_env)` в начале строки):
```powershell
pip install ipykernel
```
`ipykernel` — это пакет, который умеет превратить любой Python-интерпретатор в Jupyter-kernel (отдельный процесс, который получает код из ячейки notebook'а и возвращает результат).

### 4.2 Зарегистрировать kernel
```powershell
python -m ipykernel install --user --name isaac_env --display-name "Isaac Env (isaac_env)"
```
**Разбор параметров:**
- `--user` — регистрирует kernel только для текущего пользователя Windows (не нужны права администратора), пишет конфиг в `%APPDATA%\jupyter\kernels\isaac_env\`
- `--name isaac_env` — внутреннее техническое имя kernel (используется системой, менять не обязательно)
- `--display-name "Isaac Env (isaac_env)"` — то, что ты **увидишь в выпадающем списке** при выборе kernel в VS Code. Можно поменять на любую понятную тебе подпись, например `"Isaac Sim 6.0.1"` — на работу не влияет, это просто ярлык для человека

### 4.3 Выбрать kernel в VS Code
1. Открой `notebooks\drone_control_ardupilot.ipynb` в VS Code
2. В правом верхнем углу окна notebook'а — кнопка с текущим kernel (обычно после первого открытия покажет «Select Kernel»)
3. Нажми на неё → **Jupyter Kernel...** (не «Python Environments», а именно раздел с зарегистрированными Jupyter-ядрами) → выбери **"Isaac Env (isaac_env)"** из списка

Если `isaac_env` не появился в списке — перезапусти VS Code полностью (регистрация kernel читается при старте) или перепроверь шаг 4.2 (частая причина: команда была выполнена без активации venv, и `ipykernel` встал в системный Python вместо `isaac_env`).

### 4.4 Проверка
Запусти первую ячейку любого notebook'а (`Shift+Enter`). Если всё верно — код выполнится без ошибок `ModuleNotFoundError`. Если появляется ошибка про отсутствие модуля (`isaacsim`, `pymavlink` и т.п.) — значит выбран не тот kernel, либо пакет не доставлен в `isaac_env` (см. ниже).

---

## 5. ArduPilot SITL (внутри WSL/Ubuntu)

SITL (Software In The Loop) — это сама программа автопилота ArduCopter, скомпилированная так, чтобы физику ей поставлял не реальный дрон, а наш мост из Isaac Sim.

Открой Ubuntu (через Пуск, или набери `wsl` в PowerShell) и выполни по порядку:

### 5.1 Установить зависимости сборки
```bash
sudo apt update
sudo apt install -y git python3-pip python3-venv build-essential ccache
```
Эти пакеты нужны, чтобы скомпилировать ArduPilot из исходников: `git` — скачать код, `build-essential` — компилятор C/C++ и линковщик, `ccache` — кэш компиляции (ускоряет повторные пересборки), `python3-pip`/`python3-venv` — для python-зависимостей самого ArduPilot (mavproxy и т.д.).

### 5.2 Склонировать ArduPilot
```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```
**`--recurse-submodules`** — обязательно: репозиторий ArduPilot ссылается на несколько вложенных git-репозиториев (библиотеки типа mavlink), без этого флага они останутся пустыми папками и сборка не пройдёт.

### 5.3 Установить prerequisites-скрипт ArduPilot
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
Официальный скрипт ArduPilot, ставит все специфичные для сборки автопилота зависимости (правильные версии Python-библиотек mavlink/pymavlink/pyserial и т.д.). `-y` — не спрашивать подтверждения на каждом шаге. После него скрипт попросит перезайти в терминал (`exec bash` или переоткрыть Ubuntu) — важно это сделать, он дописывает переменные окружения (`PATH` для `ccache` и т.п.) в `~/.bashrc`.

### 5.4 Собрать SITL для квадрокоптера
```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```
`waf` — это собственная сборочная система ArduPilot (аналог `make`/`cmake`). `configure --board sitl` — говорит, что собираем не под реальную плату (Matek/Pixhawk), а под виртуальный «SITL»-борд. `./waf copter` — компилирует именно прошивку ArduCopter (квадрокоптер/мультикоптер; для самолёта было бы `./waf plane`). Результат появится в `~/ardupilot/build/sitl/bin/arducopter` — это и есть файл, который запускает `launch_sitl()` из notebook'а.

Первая сборка может занять 5-15 минут.

### 5.5 (Опционально) venv для ArduPilot Python-инструментов
В памяти проекта упоминается активация `venv-ardupilot` в скрипте запуска — если хочешь повторить в точности:
```bash
python3 -m venv ~/venv-ardupilot
source ~/venv-ardupilot/bin/activate
pip install pymavlink mavproxy
```
Это отдельное **Linux**-окружение (не путать с Windows `isaac_env`!) — нужно, если запускаешь SITL через `sim_vehicle.py` (обёртку, которая поднимает знакомое консольное окно ArduCopter с MavProxy для диагностики), как описано в `README.md`.

---

## 6. Проверка всей связки

### 6.1 Правила файрвола (один раз, PowerShell от администратора)
```powershell
New-NetFirewallRule -DisplayName "ArduPilot JSON UDP 9002" -Direction Inbound -Protocol UDP -LocalPort 9002 -Action Allow
New-NetFirewallRule -DisplayName "ArduPilot MAVLink UDP 14550" -Direction Inbound -Protocol UDP -LocalPort 14550 -Action Allow
```
WSL2 живёт в собственной виртуальной сети — трафик от SITL (внутри Ubuntu) к Isaac Sim (на Windows-хосте) идёт через виртуальный сетевой адаптер, и Windows Defender Firewall по умолчанию блокирует входящие UDP-пакеты на нестандартные порты. Эти два правила разрешают порт **9002** (физика/JSON) и **14550** (MAVLink-команды) — без них Isaac Sim никогда не получит данные от SITL, даже если всё остальное настроено верно.

### 6.2 Быстрый smoke-test
1. Открой Isaac Sim (`isaacsim` в активированном `isaac_env`), загрузи сцену с дроном `cf2x`, включи `isaacsim.code_editor.python_server`
2. В VS Code открой `notebooks/drone_control_ardupilot.ipynb`, выбери kernel `isaac_env`, выполни ячейки по порядку:
   ```python
   from isaac_bridge import *
   await bridge_up()
   launch_sitl()
   await diag()
   ```
3. `diag()` должен показывать растущий счётчик пакетов и `dt≈4.17мс` — если да, вся цепочка WSL↔Windows↔Isaac Sim работает

Если что-то не так — см. таблицу проблем в основном `README.md` (раздел про частые ошибки) и хронологию фиксов в `archive/context_ardu.md`.

---

## Итоговая карта: что где ставится

| Компонент | Где | Зачем |
|---|---|---|
| WSL2 + Ubuntu | Windows-фича + Linux-система | Запуск ArduPilot SITL |
| VS Code | Windows, обычная программа | Редактор кода + запуск notebook'ов |
| Python 3.12 | Windows | Интерпретатор для `isaac_env` |
| `isaac_env` (venv) | `C:\VSCODE\ISAAC\isaac_env` | Изолированное окружение с Isaac Sim и зависимостями моста |
| Isaac Sim 6.0.1.0 | Внутри `isaac_env` (pip-пакет) | Физический симулятор дрона |
| `ipykernel` в `isaac_env` | Внутри `isaac_env` | Позволяет VS Code запускать notebook'и именно этим интерпретатором |
| ArduPilot SITL | Внутри Ubuntu (`~/ardupilot`) | Автопилот, тот же код, что полетит на реальном Matek H743 |
