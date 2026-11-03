# Robust predicates

`predicates.c` — код Jonathan Richard Shewchuk, May 18, 1996, public domain.
Происхождение и описание указаны в начале исходного файла.

Локальная правка: в `fast_expansion_sum`, `fast_expansion_sum_zeroelim`,
`linear_expansion_sum`, `linear_expansion_sum_zeroelim` следующий элемент
массива читается только при `index < length`. Ранее выражения `e[++eindex]`
и `f[++findex]` могли читать один элемент за концом полностью заполненного
массива после обработки последней компоненты. Полученное значение дальше
не требовалось, но само чтение нарушало границы памяти.

Порядок арифметических операций сохранён. Регрессии `RobustExpansionSumTest`
проверяют одноэлементные массивы, разную длину и сокращение с малым остатком.
Исходную ошибку также обнаруживает AddressSanitizer на тесте
`PointOnSegment3DTest.SupportsExponentSpreadBoundaryWithoutEpsilon`.
