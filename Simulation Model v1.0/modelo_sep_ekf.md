# Modelo de referencia del SEP — EKF odometría + giroscopio

Modelo de simulación del Subsistema de Estimación de Pose del róver Olympus.
Sustituye al ESKF planar con cámara/GPS entregado antes.

Ref.: DRT-SEP-001 v0.1 · anteproyecto TFG · marco teórico, capítulo de fusión.

---

## 1. Qué cambió respecto de la versión base

La versión anterior no necesitaba un ajuste de parámetros: necesitaba invertirse.
En ella la IMU era la **entrada** de la predicción y la pose (cámara o GPS) era la
**medida**. El anteproyecto especifica lo contrario.

| Aspecto | Versión base | Versión SEP |
|---|---|---|
| Predicción | Integración de la IMU (acelerómetro + giro) | **Odometría skid-steer de 6 ruedas** |
| Corrección | Pose absoluta de cámara/AprilTags/GPS | **Giroscopio del MPU-9250** |
| Estados | 8: $p_x,p_y,\theta,v_x,v_y,b_\omega,b_{ax},b_{ay}$ | **5**: $p_x,p_y,\theta,\omega,b_\omega$ |
| Restricción no-holonómica | Pseudo-medida explícita | **Impuesta por la estructura** del modelo de uniciclo |
| Papel del GPS | Medida de corrección | **Verdad de terreno offline. Nunca entra al filtro** (PE-CON-001) |
| Inversión de matrices | $S$ de 3×3 en la actualización de pose | **Ninguna**: todas las medidas son escalares |
| Detección de deslizamiento | No contemplada | Discrepancia encoders–giroscopio con adaptación de $Q$ |

### Por qué se cayeron tres estados

- **$v_x, v_y$**: la velocidad la dan los encoders, no hace falta estimarla. Al
  desaparecer del estado, la restricción no-holonómica deja de ser una pseudo-medida
  y pasa a estar impuesta por el propio modelo de uniciclo. Es una restricción
  estructural, no una medida inventada.
- **$b_{ax}, b_{ay}$**: a las velocidades de este róver las aceleraciones propias son
  de orden milimétrico por segundo cuadrado y quedan enterradas en el ruido del
  MPU-9250. Los sesgos serían débilmente observables y solo añadirían estados que
  degradan el filtro. El acelerómetro se conserva como entrada auxiliar **fuera** del
  filtro, para detección de reposo y de pendiente.

---

## 2. Formulación

### Estado

$$\mathbf{x} = [\,p_x,\ p_y,\ \theta,\ \omega,\ b_\omega\,]^\top$$

### Predicción — odometría

Entrada de control $(\Delta s, \Delta\theta_{enc})$ producida por `sep_odometry` a
partir de los seis incrementos de cuentas. Con $\theta_m = \theta + \tfrac{1}{2}\omega\,\Delta t$:

$$
p_x \leftarrow p_x + \Delta s\cos\theta_m,\quad
p_y \leftarrow p_y + \Delta s\sin\theta_m,\quad
\theta \leftarrow \mathrm{wrap}(\theta + \omega\,\Delta t),\quad
\omega \leftarrow \omega_{enc},\quad
b_\omega \leftarrow b_\omega
$$

La fila de $\omega$ en el jacobiano es **nula a propósito**: la velocidad angular se
reemplaza cada paso por la que dictan los encoders y no arrastra covarianza previa.
Eso es, literalmente, «predecir con odometría».

### Corrección — giroscopio

Medida escalar $z = \omega + b_\omega$, con
$H = [0,0,0,1,\,\mu]$ donde $\mu = 1$ en reposo y $\mu = 0$ en movimiento (§ 2.2).
Forma de Joseph. $S$ es escalar, de modo que **el filtro completo no invierte ninguna
matriz** — un punto a favor del port a C embebido.

### ZARU — velocidad angular nula en reposo

Cuando las seis ruedas no cuentan, se aplica la pseudo-medida $\omega = 0$. No requiere
sensor adicional y es lo único que hace observable el sesgo del giroscopio.

---

## 2.2 Nota de diseño: por qué el sesgo solo se estima en reposo

Es el punto que hizo fallar la primera versión de este filtro y merece quedar escrito.

Durante un giro sostenido, $\omega$ y $b_\omega$ **no son separables**: cualquier
discrepancia entre encoders y giroscopio se explica igual de bien moviendo uno u otro.
Con el sesgo libre, el filtro lo usa como vertedero del error de calibración del ancho
de vía. En la verificación, con un error del 20 % en $\chi$, el sesgo derivaba a
$-0{,}058$ rad/s, $\omega$ convergía al valor **erróneo** de los encoders y el rumbo
final salía con 58° de error. El giroscopio dejaba de aportar nada.

Restringiendo la actualización del sesgo al reposo, donde la velocidad angular
verdadera es cero y la lectura del giroscopio **es** el sesgo, el error de rumbo del
mismo ensayo baja a fracciones de grado.

## 2.3 Nota de diseño: se adapta $Q$, no $R$

El anteproyecto dice «$R$ adaptativo». Con esta estructura, esa formulación queda
invertida y conviene revisarla.

Al patinar hay que desconfiar de los **encoders**, no del giroscopio. En este filtro
los encoders entran por la predicción, así que el parámetro que refleja su
incertidumbre es $Q_{\omega\omega}$, no $R$. Subir $R$ durante un deslizamiento
degradaría exactamente el sensor del que dependemos en ese momento.

El código infla `prm.s2_w_enc` con una ganancia derivada de la discrepancia
$|\omega_{gyro} - b_\omega - \omega_{enc}|$. El efecto observable es el mismo que
describe el anteproyecto —el filtro pasa a confiar en el giroscopio durante el
patinaje— pero por el lado correcto de la ecuación. **Conviene confirmarlo con Johan
antes de fijarlo en el informe.**

---

## 3. Archivos

| Archivo | Rol | codegen |
|---|---|---|
| `sep_geo_params.m` | Geometría y conversión de ticks. Fuente numérica única. | sí |
| `sep_ekf_params.m` | Sintonización del filtro. Único punto de ajuste. | sí |
| `sep_odometry.m` | 6 encoders → $(\Delta s, \Delta\theta)$. Concentra toda la geometría. | sí |
| `sep_ekf_step.m` | Núcleo del filtro. Un ciclo completo. | sí |
| `rover_params.m` | Maestro: envuelve los anteriores, añade planta, sensores, escenario, procedencia y verificación. | no |
| `build_sep_model.m` | Construye `sep_ekf_rover.slx` por código. | no |
| `run_sep_demo.m` | Punto de entrada: verdad, sensores, ejecución, métricas. | no |

**Frontera de codegen.** `rover_params.m` usa `fprintf` y `warning`, así que no puede
cruzar a Embedded Coder. Por eso no contiene valores propios: llama a
`sep_geo_params` y `sep_ekf_params`, que sí son la fuente numérica y sí son
codegen-safe. Cero duplicación y un único sitio donde editar cada número.

**Frontera de agentes.** `sep_odometry` concentra radios, ticks y ancho de vía;
`sep_ekf_step` no conoce ninguno de los tres y recibe magnitudes físicas. Esa
separación refleja el reparto entre el agente de fusión de datos y el agente de
estimación (PE-RF-006 y PE-RF-007), de modo que el modelo y la arquitectura
multiagente no se contradicen.

### Modelo Simulink generado

```
[From Workspace: enc_log ]──▶(1)  6×1 incrementos de cuentas
[From Workspace: gyro_log]──▶(2)  ┌──────────────────┐ x_est(5×1) ┌──────────────┐
[Constant      : dt = Ts ]──▶(3)  │  MATLAB Function │───────────▶│ To Workspace │
                                  │  sepEKF (x,P     │ diag(3×1)  ├──────────────┤
                                  │  persistentes)   │───────────▶│ To Workspace │
                                  └──────────────────┘            └──────────────┘
```

El GPS no aparece en el diagrama. Está solo en `run_sep_demo.m`, en las métricas y las
gráficas.

---

## 4. Uso

```matlab
>> run_sep_demo
```

`rover_params()` imprime primero la verificación de consistencia. Con los valores de
hoy avisa de dos cosas, y ambas son correctas:

1. La velocidad implicada por los 11 257 ticks/s difiere de los 0,029 m/s medidos por
   un factor de ~179. Anexo B.2.1–B.2.3.
2. Hay parámetros en estado `TBD`, de modo que los resultados no son concluyentes para
   PE-RNF-004.

Escenarios en `p.sim.scenario`: `'umbmark'` (cuadrado en ambos sentidos), `'recta'`,
`'giro'`. `p.drive.hypothesis` permite contrastar la velocidad medida (`'A'`) contra
la hipótesis de un error de unidades ×10 (`'B'`).

---

## 5. Verificación

**Advertencia de procedencia:** el algoritmo se verificó con un port independiente en
Python, no ejecutando MATLAB. Los números de abajo salen de esa verificación cruzada y
deben reproducirse ejecutando `run_sep_demo` antes de citarlos en el informe.

Condiciones: $\chi$ real 1,20 con el filtro creyendo 1,00; dispersión de radios del 1 %;
sesgo de giro 0,012 rad/s; eventos de deslizamiento del 50 % con probabilidad 0,01 por
paso; cuadrado de 2 m a 0,029 m/s.

| Métrica | Resultado |
|---|---|
| UMBmark horario | 6,2 cm = 0,78 % de 7,96 m |
| UMBmark antihorario | 12,3 cm = 1,55 % de 7,96 m |
| Sesgo de giro estimado / verdadero | 0,01215 / 0,01200 rad/s |
| Recta de 4 m, todo calibrado | 0,02 % |

### Hallazgo que reordena el Anexo B

La sensibilidad no está donde se esperaba:

| Parámetro mal calibrado | Efecto en el cuadrado | Efecto en la recta |
|---|---|---|
| $\chi$ (ancho de vía) errado de 1,00 a 2,00 | 0,78 % → 1,33 % | despreciable |
| Radio / `ticks_per_rev` con 10 % de error | 0,78 % → 0,99 % | **0,02 % → 9,98 %** |

Dos conclusiones prácticas:

1. **El giroscopio vuelve al filtro casi insensible al ancho de vía.** El rumbo ya no
   sale de la diferencia entre ruedas, así que $E_b$ deja de dominar. Esto rebaja la
   prioridad de la actividad B.2.6 respecto de lo que decía el DRT-SEP-001.
2. **El cuadrado UMBmark no mide la escala de distancia.** Un error de escala uniforme
   escala el cuadrado, que sigue cerrando. La escala se mide con la **recta**, donde el
   error del radio se transfiere uno a uno: 10 % de error en el radio son 10 % de error
   de posición. Para el presupuesto del 3 % hay que calibrar `ticks_per_rev` × `R_wheel`
   mejor que el 3 %, y eso se verifica con B.2.1 y B.2.4, no con el cuadrado.

Los tres escenarios que exige el indicador de la meta no son redundantes: **la recta
mide la escala, el cuadrado mide la orientación, y el contraste horario/antihorario
separa $E_d$ de $E_b$.** Conviene que esto quede así de explícito en el protocolo de
pruebas.

---

## 6. Pendiente

- Reproducir estos números ejecutando `run_sep_demo` en MATLAB.
- Cerrar el Anexo B y volver a correr: mientras haya parámetros `TBD`, el modelo es una
  herramienta de diseño, no una referencia de validación.
- Decidir con Johan la formulación de la adaptación ($Q$ frente a $R$, § 2.3).
- Decidir si el agente de estimación del HLC se genera con Embedded Coder desde
  `sep_ekf_step.m` o se escribe a mano en C. Si se genera, `sep_ekf_step.m` ya cumple
  las restricciones: tamaños fijos, sin asignación dinámica, sin inversión de matrices.
