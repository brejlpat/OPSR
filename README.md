# Semestralka OPSR: kulicka na desce

Projekt resi stabilizaci nelinearniho systemu kulicky na naklapene desce se dvema vstupy.

## Soubory

- `ball_plate_model.py` - nelinearni model, linearizace a kontrola riditelnosti.
- `controllers.py` - nekonecnohorizontovy LQR a MPC formulovane pres `cvxpy`.
- `simulate_ball_plate.py` - simulace nelinearniho systemu bez poruch a s poruchami.
- `animate_ball_plate_3d.py` - 3D animace pohybu kulicky na naklapene desce z ulozenych CSV vysledku.
- `semestralni_prace.md` - textovy podklad pro odevzdani.
- `results/*.csv` - ciselne vystupy simulaci po spusteni skriptu.
- `results/*.png` - grafy casovych prubehu, trajektorii a porovnani regulatoru.
- `results/*_3d.gif` - 3D animace vybranych simulaci.

## Spusteni

```powershell
.\venv\Scripts\python.exe .\simulate_ball_plate.py
```

Skript vypise rank matice riditelnosti, LQR zesileni a shrnuti simulaci. Detailni casove prubehy ulozi do `results` jako CSV a PNG.

3D animace vybraneho scenare:

```powershell
.\venv\Scripts\python.exe .\animate_ball_plate_3d.py mpc_with_disturbance
```

3D animace vsech ctyr scenaru:

```powershell
.\venv\Scripts\python.exe .\animate_ball_plate_3d.py --all
```
