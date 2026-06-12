# 差動2輪ロボットの点間移動制御設計 (v5)

**位置制御ループ1本 + 状態3つ(IDLE/GO/TURN)+ コマンド時適応λ — 暗黙的シグモイド・レーンチェンジ**

> **v5 改訂内容**: v4 の簡素構成(状態3つ、ST_ERROR なし、復旧 = 上位の set_goal 再発行)を維持したまま、**適応λのみ**を復活させた。λ の選択は set_goal 内に閉じており、ISR・状態機械は v4 と同一。TURN 復帰時の再評価は持たない——失敗時は上位の set_goal 再発行が現在姿勢で λ を選び直すため、再発行フローが再評価を自動的に兼ねる。

---

## 1. 設計の全体像

### 役割分担

| 層 | 責務 |
|---|---|
| **上位(手動シーケンス)** | 経由点の分割、後方・真横目標の処理、失敗時の再コマンド |
| **下位(本制御系)** | 受理した前方目標への、S字レーンチェンジ+直線走行のみ。S字の緩急(λ)はコマンドごとに自動選択 |

### 動作原理

レールを「**目標点Bを通り、コマンド受理時の方位に平行な直線**」と定義する。開始時の横偏差は e₀ = −d⊥ となり、臨界減衰の直線追従則

```
ω = v · (−k₁·e − k₂·θe),    k₂ = 2√k₁ = 2λ
```

がこの e₀ をS字状(シグモイド状)に収束させる。これが名目軌道である。経路生成器・軌跡バッファは存在しない。

横方向ダイナミクスは弧長 s に対する線形2次系 e″ + 2λe′ + λ²e = 0 であり、解は

```
e(s) = e₀ (1 + λs) e^(−λs)
```

主要な解析値:

| 量 | 式 |
|---|---|
| 整定距離 | L ≈ 5/λ |
| ピーク方位偏差 | θe_max ≈ 0.37·\|e₀\|·λ |
| ピーク曲率 | κ_max = \|e₀\|·λ² |

ゲインを v でスケールしているため、S字の空間形状は速度・加減速に依存しない。

### 状態機械(v4 と同一)

```
              受理(λ選択込み)              到達
 IDLE ──set_goal()──→ GO ──────────────────→ IDLE (成否フラグ付き)
                      ↑│
        |θe| < TH_TOL ││ |θe| > TH_FAIL
                      │↓
                     TURN (レール方位へ回頭するだけ)
```

- **GO**: 唯一の走行状態。台形速度 + 追従則
- **TURN**: フォールバック専用。レール方位 θ_line へその場回頭し、同じゲインのまま GO に戻る。レールは再定義しない
- **復旧 = set_goal() 再発行**: 到達成否フラグ(横偏差 \|e\| < LAT_TOL)を上位が見て、必要なら同じ目標点を再コマンドする。set_goal は現在の位置・方位からレールを引き直し、**そのときの横偏差・残距離で λ を選び直す**。すなわち復旧と再適応が単一のフローで実現される

---

## 2. 受理判定と適応λ

コマンド受理時、現在方位 h = (cos θ₀, sin θ₀) で B−A を分解:

```
d∥ = (B−A)·h          (前進距離)
d⊥ = h × (B−A)        (横オフセット、左正)
```

λ の制約は2つ:

```
(a) 方位偏差上限:  0.37·|d⊥|·λ ≤ θ_allow   →  λ ≤ λ_hi = 2.7·θ_allow / |d⊥|
(b) 整定距離:      5/λ ≤ α·d∥               →  λ ≥ λ_lo = 5 / (α·d∥)
```

```
受理 ⇔  d∥ ≥ D_FWD_MIN  かつ  λ_lo ≤ λ_hi
選択:   λ = clamp(LAMBDA_DEFAULT, λ_lo, λ_hi)
```

- おおよその受理条件は **d∥ ≳ 2.3·\|d⊥\|/θ_allow**(例: θ_allow = 0.3 rad なら、横 0.2 m に前進約 1.5 m 必要)
- 横オフセットが大きいほど λ は小さく(S字が長く緩く)選ばれ、ピーク方位偏差 θe_max ≤ θ_allow が常に保証される
- 直進のみ(\|d⊥\| ≈ 0)では λ = LAMBDA_DEFAULT。S字は退化して純直進になる(同一制御則の特殊ケース)
- 不受理(後方・真横・横過大)は false を返し、上位が分割する

---

## 3. 実装(C)

```c
typedef enum { ST_IDLE, ST_GO, ST_TURN } state_t;

/* --- 定数 ---
   LAMBDA_DEFAULT : λ 既定値 [1/m] (S字長 ≈ 5/λ)
   TH_ALLOW       : 名目ピーク方位偏差の上限 [rad]
   ALPHA          : 整定距離マージン (例 0.8)
   D_FWD_MIN      : 受理する最小前進距離(レール定義の安定下限)
   E_EPS          : 横オフセットの実質ゼロ判定(測位ノイズ程度)
   TH_FAIL        : フォールバック閾値 (TH_ALLOW の 2〜3 倍)
   ほか: V_MAX, A_MAX, W_GO_MAX, W_MAX, K_TH, TH_TOL,
         POS_TOL, LAT_TOL, DT, TREAD                          */

typedef struct {
    float ax, ay;      /* レールアンカー A' = B − d∥·u */
    float ux, uy;      /* レール方向(= 出発時方位)    */
    float th_line;
    float len;         /* = d∥ */
    float k1, k2;      /* 適応ゲイン(コマンドごとに設定) */
} cmd_t;

volatile cmd_t   cmd;
volatile state_t state = ST_IDLE;
volatile bool    arrived_ok = false;   /* 到達成否(上位が読む) */
static float     v = 0.0f;

/* ---- 目標点セット(ISR外)。false = 不受理 → 上位で分割 ---- */
bool set_goal(float gx, float gy)
{
    float hx = cosf(pose.th), hy = sinf(pose.th);
    float dx = gx - pose.x,   dy = gy - pose.y;
    float d_fwd =  dx*hx + dy*hy;
    float d_lat = -dx*hy + dy*hx;

    if (d_fwd < D_FWD_MIN) return false;

    /* 適応λ: 横オフセットが大きいほどS字を長く(緩く)取る */
    float e0     = fabsf(d_lat);
    float lam_hi = (e0 > E_EPS) ? 2.7f * TH_ALLOW / e0 : LAMBDA_DEFAULT;
    float lam_lo = 5.0f / (ALPHA * d_fwd);
    if (lam_lo > lam_hi) return false;             /* 横過大 → 上位で分割 */
    float lam = clampf(LAMBDA_DEFAULT, lam_lo, lam_hi);

    cmd.ux = hx;  cmd.uy = hy;
    cmd.th_line = pose.th;
    cmd.len = d_fwd;
    cmd.ax  = gx - d_fwd * hx;
    cmd.ay  = gy - d_fwd * hy;
    cmd.k1  = lam * lam;
    cmd.k2  = 2.0f * lam;

    v = 0.0f;
    arrived_ok = false;
    state = ST_GO;                       /* θe(0)=0: 回頭不要で即直進 */
    return true;
}

/* ---- タイマーISR(周期 DT、測位系の更新周期に同期) ---- */
void control_isr(void)
{
    /* pose は一括スナップショットで読むこと */
    float px  = pose.x - cmd.ax,  py = pose.y - cmd.ay;
    float s   =  px*cmd.ux + py*cmd.uy;          /* 進行距離 */
    float e   = -px*cmd.uy + py*cmd.ux;          /* 横偏差   */
    float the = wrap_pi(pose.th - cmd.th_line);  /* 方位偏差 */
    float w = 0.0f;

    switch (state) {

    case ST_GO: {
        float d_rem = cmd.len - s;
        v = fminf(fminf(v + A_MAX*DT, V_MAX),
                  sqrtf(2.0f*A_MAX*fmaxf(d_rem, 0.0f)));
        w = clampf(v * (-cmd.k1*e - cmd.k2*the), -W_GO_MAX, W_GO_MAX);

        if (fabsf(the) > TH_FAIL) {              /* フォールバック */
            v = 0.0f;
            state = ST_TURN;
        }
        if (d_rem < POS_TOL || s > cmd.len) {    /* 到達/通過 */
            v = 0.0f;  w = 0.0f;
            arrived_ok = (fabsf(e) < LAT_TOL);   /* false なら上位が再コマンド */
            state = ST_IDLE;
        }
        break;
    }

    case ST_TURN:                                /* レール方位へ回頭して復帰 */
        w = clampf(-K_TH * the, -W_MAX, W_MAX);
        if (fabsf(the) < TH_TOL) { state = ST_GO; }
        break;

    case ST_IDLE:
    default:
        v = 0.0f;  w = 0.0f;
        break;
    }

    set_wheel_vel(v - 0.5f*TREAD*w,   /* 左輪 */
                  v + 0.5f*TREAD*w);  /* 右輪 */
}
```

v4 からの差分は cmd への k1, k2 追加と、set_goal 内の λ 選択ブロック(6行)のみ。ISR・状態機械は同一。

---

## 4. パラメータ

| 定数 | 決め方 |
|---|---|
| **LAMBDA_DEFAULT** | 標準のS字長さ仕様 L から λ = 5/L。例: L = 0.5 m → λ = 10 |
| **TH_ALLOW** | 「平行移動に見える」ピーク方位偏差の仕様値(例: 0.3 rad ≈ 17°) |
| **ALPHA** | 整定距離マージン。例: 0.8 |
| **TH_FAIL** | TH_ALLOW の 2〜3 倍。名目動作では絶対に届かないマージンを確保 |
| **W_GO_MAX** | 名目ピーク曲率を通す: W_GO_MAX ≥ V_MAX·κ_max。κ_max = \|e₀\|λ² は λ ≤ λ_hi より \|e₀\|·LAMBDA_DEFAULT² で上から押さえられる。同時にゲイン異常時の物理上限 |
| **K_TH, W_MAX, TH_TOL** | 回頭の P ゲイン・角速度上限・完了閾値(例: 5°) |
| **POS_TOL, LAT_TOL** | 到達判定(縦・横)。測位ノイズに応じて |
| **D_FWD_MIN, E_EPS** | レール定義の安定下限・横ゼロ判定。測位ノイズに応じて |

チューニングの主対象は LAMBDA_DEFAULT・TH_ALLOW・TH_FAIL の3つ。E_MAX/D_MIN のような封筒定数は不要(λ 選択が肩代わり)。

---

## 5. 実装上の注意(最小限)

- **pose の一括スナップショット**: x, y, θ を別タイミングで読むと ω にスパイクが乗る。ダブルバッファ等で原子的に
- **sqrt 回避(任意)**: `d_rem < v*v/(2*A_MAX)` の比較式に置換可能。三角関数・除算は set_goal のみ
- **ノイズ性スパイクで TURN が誤発動する場合(任意)**: TH_FAIL 超過の連続周期カウント(3行)を足す
- **v→0 で操舵消失**: 到達間際は e がすでに収束済みのため実害なし
- **TURN 復帰の収束不足**: 同じゲインで復帰するため、残距離が足りなければ横偏差が残ったまま到達し arrived_ok = false となる。上位の set_goal 再発行が現在姿勢・残距離で λ を選び直すので、これが実質的な再評価・復旧動作になる(専用ロジック不要)

---

## 付録: 構成の変遷

| 版 | 構成 | 備考 |
|---|---|---|
| v1 | A→Bレール + TURN+GO | e₀ が常に 0 になり名目S字が出ない欠陥 |
| v2 | LANE / TURN+GO の2モード + 適応λ | レール定義を修正 |
| v3 | LANE 唯一走行 + TURN フォールバック + 再評価 + ERROR | 機構過多 |
| v4 | 固定λ + 静的封筒 + 3状態 | 最小構成 |
| **v5** | **v4 + コマンド時適応λ** | 適応は set_goal に閉じ、ISR は v4 と同一 |
