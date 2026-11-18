# third-order-control-analysis

**Context:** Course / personal project — Universidad Nacional de Ingeniería (Curso: Control Moderno MT227-C)
**Date:** 2026
**Tools:** MATLAB, Simulink, Proteus, Osciloscopio (Circuito físico)
**Status:** Completed / documented retrospectively

## Summary
Este proyecto aborda el análisis, diseño y simulación de un sistema de control en espacio de estados para una planta electrónica de tercer orden. Incluye el rediseño de un circuito inicialmente no controlable, la síntesis de un controlador por realimentación de estados con acción integral, y el diseño de observadores de Luenberger (orden completo y reducido) para estimar variables no medibles, validando el principio de separación.

## Objective
- Obtener la representación en espacio de estados de una planta de tercer orden (circuito con amplificadores operacionales, resistencias y capacitores).
- Analizar las propiedades fundamentales del sistema: estabilidad (eigenvalores), controlabilidad (matriz de Kalman) y observabilidad.
- Diseñar un controlador por realimentación de estados con acción integral y pre-filtro (\(N=1\)) para el seguimiento de referencias escalón.
- Diseñar observadores de estado de orden completo y de orden reducido (Luenberger) para estimar los estados no medidos.
- Validar el diseño mediante simulaciones en MATLAB/Simulink y simulación de circuitos en Proteus, contrastando con pruebas físicas en osciloscopio.

## What I did
- Designed ...
- Programmed ...
- Simulated ...
- Validated ...

## Results / Evidence
- **Key result or metric:** El sistema rediseñado resultó ser estable (polos en \(-10.39\), \(-9.71\), \(-4.28\)), completamente controlable (rango = 3) y observable. Se verificó matemáticamente y en simulación que los polos del controlador y del observador son independientes (Principio de Separación), logrando un error en estado estacionario nulo ante una entrada escalón.
- **Photos/videos:** *(Agregar capturas de pantalla de las respuestas al escalón en Simulink, diagramas de Bode, o fotos del osciloscopio del circuito físico)*.
- **Report:** [Informe del Proyecto (PDF)](docs/report.pdf)

## 📂 Contenido del Repositorio

*   **[Documentación (`docs/)`](docs/)**: Informe completo en PDF (`report.pdf`) con el marco teórico y desarrollo matemático.
*   **[Análisis de Estabilidad (`stability-analysis/)`](stability-analysis/)**: Scripts y resultados del cálculo de eigenvalores, matrices de controlabilidad y observabilidad.
*   **[Controlador y Observador (`controller-observer/)`](controller-observer/)**: Diseño de las ganancias \(K\) (método ITAE/Ackermann) y \(L\) (observadores de orden completo y reducido).
*   **[Código Fuente (`src/)`](src/)**: Scripts en MATLAB y modelos de Simulink utilizados para las simulaciones.
*   **[Resultados (`results/)`](results/)**: Gráficas de respuesta al escalón, esfuerzos de control y validación en Proteus.

## 🚧 Limitaciones del Proyecto
Al tratarse de un laboratorio académico de control, el sistema presenta las siguientes restricciones:

*   **Modelo idealizado:** El diseño matemático asume componentes ideales (amplificadores operacionales sin offset, valores exactos de resistencias y capacitores). En la práctica, las tolerancias de los componentes y las variaciones de temperatura pueden alterar los polos del sistema.
*   **Sensibilidad al ruido:** El observador de orden reducido, aunque computacionalmente más eficiente, es altamente sensible al ruido de medición, ya que requiere el cálculo de la derivada de la salida (que en un entorno real amplifica el ruido).
*   **Entorno de validación:** La planta fue validada en simuladores (MATLAB/Simulink, Proteus) y en un circuito físico de laboratorio, sin estar expuesta a perturbaciones externas severas o ruido electromagnético industrial.

## 🚀 Proyecciones a Futuro
Para escalar este proyecto desde un laboratorio académico hacia una aplicación de ingeniería real, se proyecta:

*   **Implementación Digital:** Migrar el controlador y los observadores continuos a un entorno digital (microcontrolador, DSP o FPGA), implementando las ecuaciones en diferencias y evaluando el periodo de muestreo adecuado.
*   **Control Óptimo (LQR):** Sustituir la asignación de polos por el diseño de un Regulador Lineal Cuadrático (LQR) para optimizar el compromiso entre el esfuerzo de control y la rapidez de respuesta.
*   **Filtrado de Ruido:** Implementar un Filtro de Kalman en lugar del observador de Luenberger clásico para manejar de forma robusta el ruido de los sensores reales y las incertidumbres del modelo.
*   **Robustez:** Analizar la sensibilidad del sistema ante variaciones paramétricas (variación de R y C) y diseñar controladores robustos (ej. \(H_\infty\)) si el sistema debe operar en condiciones adversas.

---
## 📊 Simulaciones y Evidencia
*(Aquí puedes agregar imágenes de las respuestas al escalón obtenidas en MATLAB y las capturas del osciloscopio del circuito físico)*

## Authors / credits
- Lorenzo A. Mendoza
- Luciano A. Mezones Rodas, Kevin A. Ramirez Cruz
