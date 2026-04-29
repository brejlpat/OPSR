# Semestrálka OPSŘ: kulička na desce

Projekt řeší stabilizaci nelineárního systému kuličky na naklápěné desce se dvěma vstupy.

## Soubory

- `ball_plate_model.py` - nelineární model, linearizace a kontrola řiditelnosti.
- `controllers.py` - nekonečnohorizontový LQR a MPC formulované přes `cvxpy`.
- `simulate_ball_plate.py` - simulace nelineárního systému bez poruch a s poruchami.
- `animate_ball_plate_3d.py` - 3D animace pohybu kuličky na naklápěné desce z uložených CSV výsledků.
- `semestralni_prace.md` - textový podklad pro odevzdání.
- `results/*.csv` - číselné výstupy simulací po spuštění skriptu.
- `results/*.png` - grafy časových průběhů, trajektorií a porovnání regulátorů.
- `results/*_3d.gif` - 3D animace vybraných simulací.

## Spuštění

```powershell
.\venv\Scripts\python.exe .\simulate_ball_plate.py
```

Skript vypíše rank matice řiditelnosti, LQR zesílení a shrnutí sumulací. Detailní časové průběhy uloží do `results` jako CSV a PNG. 
3D animace vybraného scénáře:

```powershell
.\venv\Scripts\python.exe .\animate_ball_plate_3d.py mpc_with_disturbance
```

3D animace všech čtyř scénářů:

```powershell
.\venv\Scripts\python.exe .\animate_ball_plate_3d.py --all
```
