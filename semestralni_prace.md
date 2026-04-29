# Semestrální práce: kulička na desce

## 1. Popis řešené úlohy

Cílem úlohy je navrhnout řízení pro kuličku pohybující se po naklápěné desce. Deska je ovládána ve dvou nezávislých osách, proto má systém dva řídicí vstupy. Úlohou regulátoru je stabilizovat kuličku v počátku souřadnic desky, tedy v bodě `p_x = 0`, `p_y = 0`, a potlačit vliv poruch působících jako neznámé horizontální zrychlení.

Typické fyzikální interpretace poruch jsou nepřesnosti modelu, nerovnost desky, chyba kalibrace nulového náklonu nebo vnější impuls do kuličky.

## 2a. Nelineární systém a matematický model

Stavový vektor je

```text
x = [p_x, p_y, v_x, v_y]
```

kde p_x, p_y jsou polohy kuličky na desce a v_x, v_y jsou rychlosti. Řídicí vstup je

```text
u = [theta_x, theta_y]^T
```

kde theta_x, theta_y jsou náklony desky v radiánech. Pro kuličku valící se bez prokluzu je zrychlení po nakloněné rovině zmenšeno faktorem 5/7, proto je použit model

```text
dot(p_x) = v_x
dot(p_y) = v_y
dot(v_x) = (5/7) g sin(theta_x) - b v_x + d_x
dot(v_y) = (5/7) g sin(theta_y) - b v_y + d_y
```

kde g = 9.81 m/s^2, b = 0.25 s^-1 je lineární tlumení a d_x, d_y jsou poruchová zrychlení. Nelinearita je dána členy sin(theta_x) a sin(theta_y).

V kódu je model implementován v souboru ball_plate_model.py, funkce continuous_dynamics.

## 2b. Stavy, vstupy a poruchy

Stavy:

- `p_x [m]` - poloha kuličky v ose x,
- `p_y [m]` - poloha kuličky v ose y,
- `v_x [m/s]` - rychlost kuličky v ose x,
- `v_y [m/s]` - rychlost kuličky v ose y.

Ridici vstupy:

- `theta_x [rad]` - náklon desky působící v ose x,
- `theta_y [rad]` - náklon desky působící v ose y.

Uvažované poruchy:
- konstantní bias zrychlení, reprezentující chybu kalibrace nebo nerovnost desky,
- krátkodobé impulsní poruchy, reprezentující vnější postrčení kuličky.

## 3a. Pracovní bod a linearizace

Pracovní bod pro stabilizaci je klidová poloha uprostřed desky:

```text
x0 = [0, 0, 0, 0]
u0 = [0, 0]^T
d0 = [0, 0]^T
```

Linearizace je provedena jako první Taylorův rozvoj nelineárního modelu v pracovním bodě. Pro nelineární zápis `dot(x) = f(x,u)` jsou matice lineárního modelu odvozeny jako Jacobiány

```text
A = df/dx |_(x0,u0)
B = df/du |_(x0,u0)
```

Protože `sin(theta) ~= theta` pro malé úhly a `cos(0) = 1`, dostaneme spojitý lineární model

```text
dot(x) = A x + B u
```

s maticemi

```text
A = [[0, 0, 1, 0],
     [0, 0, 0, 1],
     [0, 0, -b, 0],
     [0, 0, 0, -b]]

B = [[0, 0],
     [0, 0],
     [(5/7)g, 0],
     [0, (5/7)g]]
```

Pro diskrétni regulaci je model diskretizován metodou zero-order hold s periodou vzorkování `T_s = 0.05 s`. Tato metoda předpokládá, že řídicí vstup je mezi dvěma vzorky konstantní. Diskrétní matice `A_d`, `B_d` jsou v kódu vypočteny pomocí exponenciální matice rozšířené blokové matice, což odpovídá přesně diskretizaci lineárního spojitého modelu pro konstantní vstup v intervalu vzorkování.

## 3b. Ověření řiditelnosti

Řiditelnost se ověřuje rankem matice

```text
C = [B, AB, A^2B, A^3B].
```

Systém ma čtyři stavy, proto je linearizovaný model řiditelný, pokud `rank(C) = 4`. Skript `simulate_ball_plate.py` vypisuje rank pro spojitý i diskrétní model. Při zvolených parametrech vychází v obou případech rank `4`, tedy systém je v pracovním bodě řiditelný.

## 4. Návrh LQR s nekonečným horizontem

LQR je navržen pro diskrétní linearizovaný model. Minimalizované kritérium je

```text
J = sum_{k=0}^{infty} (x_k^T Q x_k + u_k^T R u_k)
```

se zvolenými váhami

```text
Q = diag(180, 180, 8, 8)
R = diag(0.8, 0.8)
```

Vyšší váhy na polohách v Q nutí regulátor rychle vracet kuličku do středu desky. Matice R omezuje agresivitu naklápění. Zpětnovazební zákon má tvar

```text
u_k = -K x_k
```

kde K je zisk vypočtený řešením diskrétní algebraické Riccatiho rovnice. Vstup je v simulaci saturován na +-10 deg, aby odpovídal fyzikálně reálné desce.

## 5. MPC problém stabilizace s omezenými vstupy a stavy

Jako druhá metoda je použito MPC nad diskrétním linearizovaným modelem. Optimalizační problém pro horizont N = 25 je

```text
min sum_{k=0}^{N-1} ((x_k-r)^T Q (x_k-r) + u_k^T R u_k)
    + (x_N-r)^T P (x_N-r)

s.t. x_0 = aktuální stav
     x_{k+1} = A_d x_k + B_d u_k
     -u_max <= u_k <= u_max
     -x_max <= x_k <= x_max
```

kde terminální matice P je převzata z LQR Riccatiho rovnice. Omezení jsou

```text
|theta_x|, |theta_y| <= 10 deg
|p_x|, |p_y| <= 0.18 m
|v_x|, |v_y| <= 0.8 m/s
```

Optimalizační problém je kvadratický program řešený knihovnou cvxpy se solverem OSQP. Implementace je ve třídě LinearMPC v souboru controllers.py.

Oproti LQR je hlavní výhodou MPC přímé zahrnutí omezení do optimalizační úlohy. LQR dává jednoduchý lineární zpětnovazební zákon u_k = -K x_k, ale samotný návrh LQR nepracuje s omezeními vstupu ani stavu; v simulaci se proto vstup pouze dodatečně saturyje. MPC naopak omezení u_max a x_max respektuje už při výpočtu optimální posloupnosti vstupů v predikčním horizontu.

## 6. Simulace řízení nelineárního systému

Simulace je provedena na nelineárním modelu, ne pouze na linearizaci. Integrace probíhá metodou RK4 s krokem 0.05 s. Počáteční stav je

```text
x(0) = [0.12, -0.08, 0, 0].
```

Spuštění:

```powershell
.\venv\Scripts\python.exe .\simulate_ball_plate.py
```

Vystupy se uloží do složky `results`:

- `lqr_without_disturbance.csv`,
- `lqr_with_disturbance.csv`,
- `mpc_without_disturbance.csv`,
- `mpc_with_disturbance.csv`.

Ke každé simulaci se navíc generují obrázky:

- `*_timeseries.png` - časové průběhy poloh, rychlosti a vstupu,
- `*_trajectory.png` - trajektorie kuličky v rovině desky,
- `*_disturbance.png` - průběh poruch pro simulace s poruchou,
- `comparison_without_disturbance.png` a `comparison_with_disturbance.png` - porovnání LQR a MPC.

Pro názorné zobrazení pohybu kuličky je doplněn také skript animate_ball_plate_3d.py, který z uložených CSV výsledků vytváří 3D animaci naklápěné desky a kuličky. Výstupem je soubor *_3d.gif a statický náhled *_3d_final.png ve složce results.

### 6a. Bez poruch

Bez poruch oba regulátory stabilizují kuličku do středu desky. LQR je jednodušší a má analytický výpočet vstupu. MPC navíc přímo respektuje omezení vstupu a stavové limity v predikčním horizontu.

Při simulaci ze stavu [0.12, -0.08, 0, 0] vyšel pro LQR i MPC konečný polohový norm prakticky nulový. Maximální souřadnice polohy zůstala 0.12 m, což odpovídá počáteční výchylce, a maximální náklon dosáhl saturace 10 deg.

### 6b. S poruchami

S poruchami je do modelu přidáno malé konstantní zrychlení a dva krátké impulsy. Protože LQR ani MPC v této verzi neobsahují integrální složku ani estimátor konstantní poruchy, může zůstat malá ustálená odchylka. Regulátor přesto systém stabilizuje a po impulsních poruchách vrací kuličku zpět k počátku.

Pro zvolenou poruchu vyšel konečný polohový norm přibližně 0.00052 m pro LQR i MPC. Vstup zůstal omezen saturací +-10 deg.

## 7. Zaver

Byl sestaven nelineární model kuličky na desce se dvěma vstupy, linearizován v klidovém pracovním bodě a diskretizován pro číslicovou regulaci. Řiditelnost linearizovaného modelu byla ověřena rankem matice řiditelnosti. Pro stabilizaci byl navržen nekonečnohorizontový LQR a MPC s omezením vstupu a stavu. Simulace jsou provedeny na nelineárním modelu pro případ bez poruch i s poruchami.
