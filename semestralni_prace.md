# Semestralni prace: kulicka na desce

## 1. Popis resene ulohy

Cilem ulohy je navrhnout rizeni pro kulicku pohybujici se po naklapene desce. Deska je ovladana ve dvou nezavislych osach, proto ma system dva ridici vstupy. Ulohou regulatoru je stabilizovat kulicku v pocatku souradnic desky, tedy v bode `p_x = 0`, `p_y = 0`, a potlacit vliv poruch pusobicich jako nezname horizontalni zrychleni.

Typicke fyzikalni interpretace poruch jsou nepresnosti modelu, nerovnost desky, chyba kalibrace nuloveho naklonu nebo vnejsi impuls do kulicky.

## 2a. Nelinearni system a matematicky model

Stavovy vektor je

```text
x = [p_x, p_y, v_x, v_y]
```

kde `p_x`, `p_y` jsou polohy kulicky na desce a `v_x`, `v_y` jsou rychlosti. Ridici vstup je

```text
u = [theta_x, theta_y]^T
```

kde `theta_x`, `theta_y` jsou naklony desky v radianech. Pro kulicku valici se bez prokluzu je zrychleni po naklonene rovine zmenseno faktorem `5/7`, proto je pouzit model

```text
dot(p_x) = v_x
dot(p_y) = v_y
dot(v_x) = (5/7) g sin(theta_x) - b v_x + d_x
dot(v_y) = (5/7) g sin(theta_y) - b v_y + d_y
```

kde `g = 9.81 m/s^2`, `b = 0.25 s^-1` je linearni tlumeni a `d_x`, `d_y` jsou poruchova zrychleni. Nelinearita je dana cleny `sin(theta_x)` a `sin(theta_y)`.

V kodu je model implementovan v souboru `ball_plate_model.py`, funkce `continuous_dynamics`.

## 2b. Stavy, vstupy a poruchy

Stavy:

- `p_x [m]` - poloha kulicky v ose x,
- `p_y [m]` - poloha kulicky v ose y,
- `v_x [m/s]` - rychlost kulicky v ose x,
- `v_y [m/s]` - rychlost kulicky v ose y.

Ridici vstupy:

- `theta_x [rad]` - naklon desky pusobici v ose x,
- `theta_y [rad]` - naklon desky pusobici v ose y.

Usetrovane poruchy:

- konstantni bias zrychleni, reprezentujici chybu kalibrace nebo nerovnost desky,
- kratkodobe impulsni poruchy, reprezentujici vnejsi postrceni kulicky.

## 3a. Pracovni bod a linearizace

Pracovni bod pro stabilizaci je klidova poloha uprostred desky:

```text
x0 = [0, 0, 0, 0]
u0 = [0, 0]^T
d0 = [0, 0]^T
```

Linearizace je provedena jako prvni Tayloruv rozvoj nelinearniho modelu v pracovnim bode. Pro nelinearni zapis `dot(x) = f(x,u)` jsou matice linearniho modelu odvozeny jako Jacobiany

```text
A = df/dx |_(x0,u0)
B = df/du |_(x0,u0)
```

Protoze `sin(theta) ~= theta` pro male uhly a `cos(0) = 1`, dostaneme spojity linearni model

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

Pro diskretni regulaci je model diskretizovan metodou zero-order hold s periodou vzorkovani `T_s = 0.05 s`. Tato metoda predpoklada, ze ridici vstup je mezi dvema vzorky konstantni. Diskretni matice `A_d`, `B_d` jsou v kodu vypocteny pomoci exponencialni matice rozsirene blokove matice, coz odpovida presne diskretizaci linearniho spojiteho modelu pro konstantni vstup v intervalu vzorkovani.

## 3b. Overeni riditelnosti

Riditelnost se overuje rankem matice

```text
C = [B, AB, A^2B, A^3B].
```

System ma ctyri stavy, proto je linearizovany model riditelny, pokud `rank(C) = 4`. Skript `simulate_ball_plate.py` vypisuje rank pro spojity i diskretni model. Pri zvolenych parametrech vychazi v obou pripadech rank `4`, tedy system je v pracovnim bode riditelny.

## 4. Navrh LQR s nekonecnym horizontem

LQR je navrzen pro diskretni linearizovany model. Minimalizovane kriterium je

```text
J = sum_{k=0}^{infty} (x_k^T Q x_k + u_k^T R u_k)
```

se zvolenymi vahami

```text
Q = diag(180, 180, 8, 8)
R = diag(0.8, 0.8)
```

Vyssi vahy na polohach v `Q` nuti regulator rychle vracet kulicku do stredu desky. Matice `R` omezuje agresivitu naklapeni. Zpetnovazebni zakon ma tvar

```text
u_k = -K x_k
```

kde `K` je zisk vypocteny resenim diskretni algebraicke Riccatiho rovnice. Vstup je v simulaci saturovan na `+-10 deg`, aby odpovidal fyzikalne realne desce.

## 5. MPC problem stabilizace s omezenymi vstupy a stavy

Jako druha metoda je pouzito MPC nad diskretnim linearizovanym modelem. Optimalizacni problem pro horizont `N = 25` je

```text
min sum_{k=0}^{N-1} ((x_k-r)^T Q (x_k-r) + u_k^T R u_k)
    + (x_N-r)^T P (x_N-r)

s.t. x_0 = aktualni stav
     x_{k+1} = A_d x_k + B_d u_k
     -u_max <= u_k <= u_max
     -x_max <= x_k <= x_max
```

kde terminalni matice `P` je prevzata z LQR Riccatiho rovnice. Omezeni jsou

```text
|theta_x|, |theta_y| <= 10 deg
|p_x|, |p_y| <= 0.18 m
|v_x|, |v_y| <= 0.8 m/s
```

Optimalizacni problem je kvadraticky program reseny knihovnou `cvxpy` se solverem OSQP. Implementace je ve tride `LinearMPC` v souboru `controllers.py`.

Oproti LQR je hlavni vyhodou MPC prime zahrnuti omezeni do optimalizacni ulohy. LQR dava jednoduchy linearni zpetnovazebni zakon `u_k = -K x_k`, ale samotny navrh LQR nepracuje s omezenimi vstupu ani stavu; v simulaci se proto vstup pouze dodatecne saturuje. MPC naopak omezeni `u_max` a `x_max` respektuje uz pri vypoctu optimalni posloupnosti vstupu v predikcnim horizontu.

## 6. Simulace rizeni nelinearniho systemu

Simulace je provedena na nelinearnim modelu, ne pouze na linearizaci. Integrace probiha metodou RK4 s krokem `0.05 s`. Pocatecni stav je

```text
x(0) = [0.12, -0.08, 0, 0].
```

Spusteni:

```powershell
.\venv\Scripts\python.exe .\simulate_ball_plate.py
```

Vystupy se ulozi do slozky `results`:

- `lqr_without_disturbance.csv`,
- `lqr_with_disturbance.csv`,
- `mpc_without_disturbance.csv`,
- `mpc_with_disturbance.csv`.

Ke kazde simulaci se navic generuji obrazky:

- `*_timeseries.png` - casove prubehy poloh, rychlosti a vstupu,
- `*_trajectory.png` - trajektorie kulicky v rovine desky,
- `*_disturbance.png` - prubeh poruch pro simulace s poruchou,
- `comparison_without_disturbance.png` a `comparison_with_disturbance.png` - porovnani LQR a MPC.

Pro nazorne zobrazeni pohybu kulicky je doplnen take skript `animate_ball_plate_3d.py`, ktery z ulozenych CSV vysledku vytvari 3D animaci naklapene desky a kulicky. Vystupem je soubor `*_3d.gif` a staticky nahled `*_3d_final.png` ve slozce `results`.

### 6a. Bez poruch

Bez poruch oba regulatory stabilizuji kulicku do stredu desky. LQR je jednodussi a ma analyticky vypocet vstupu. MPC navic primo respektuje omezeni vstupu a stavove limity v predikcnim horizontu.

Pri simulaci ze stavu `[0.12, -0.08, 0, 0]` vysel pro LQR i MPC konecny polohovy norm prakticky nulovy. Maximalni souradnice polohy zustala `0.12 m`, coz odpovida pocatecni vychylce, a maximalni naklon dosahl saturace `10 deg`.

### 6b. S poruchami

S poruchami je do modelu pridano male konstantni zrychleni a dva kratke impulsy. Protoze LQR ani MPC v teto verzi neobsahuji integralni slozku ani estimator konstantni poruchy, muze zustat mala ustalena odchylka. Regulator presto system stabilizuje a po impulsnich poruchach vraci kulicku zpet k pocatku.

Pro zvolenou poruchu vysel konecny polohovy norm priblizne `0.00052 m` pro LQR i MPC. Vstup zustal omezen saturaci `+-10 deg`.

## 7. Zaver

Byl sestaven nelinearni model kulicky na desce se dvema vstupy, linearizovan v klidovem pracovnim bode a diskretizovan pro cislicovou regulaci. Riditelnost linearizovaneho modelu byla overena rankem matice riditelnosti. Pro stabilizaci byl navrzen nekonecnohorizontovy LQR a MPC s omezenim vstupu a stavu. Simulace jsou provedeny na nelinearnim modelu pro pripad bez poruch i s poruchami.

Rozsireni prace by mohlo zahrnovat integralni slozku nebo rozsireny stavovy observer pro kompenzaci konstantnich poruch, pripadne presnejsi model desky vcetne dynamiky servomotoru a vazby mezi osami.
