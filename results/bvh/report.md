# Замеры поиска через BVH

Сравниваются готовые функции `FindIntersectionsBruteForce` и `FindIntersectionsBvh`.
Время BVH включает построение дерева, подготовку боксов, сбор кандидатов и проверки пар.
Создание сцены и сравнение масок происходят вне измеряемого участка.
Для каждого размера и сценария: один прогрев, затем три повтора; в таблице медиана.
Порядок алгоритмов чередуется. Маски совпали при прогреве и во всех измеренных повторах.

Сцены детерминированы, seed не используется. Все треугольники лежат в XY:
вершины `(x, 0, 0)`, `(x+1, 0, 0)`, `(x, 1, 0)`.
`separated`: x=4,8,12,...; `paired`: x=4,4,8,8,...;
`dense`: x=0 для всех фигур. В paired отдельные пары совпадают, в dense совпадают все фигуры.

| Сцена | N | Пар полного перебора | Кандидатов BVH | Проверок треугольников BVH | Перебор, мс | BVH, мс | Перебор / BVH |
|---|---:|---:|---:|---:|---:|---:|---:|
| separated | 128 | 8128 | 192 | 0 | 55.574 | 0.060 | 926.23 |
| paired | 128 | 8128 | 192 | 64 | 50.508 | 0.167 | 302.81 |
| dense | 128 | 8128 | 8128 | 8128 | 12.185 | 11.834 | 1.03 |
| separated | 512 | 130816 | 768 | 0 | 941.282 | 0.263 | 3583.11 |
| paired | 512 | 130816 | 768 | 256 | 917.290 | 0.723 | 1269.43 |
| dense | 512 | 130816 | 130816 | 130816 | 167.434 | 155.540 | 1.08 |
| separated | 1024 | 523776 | 1536 | 0 | 2649.968 | 0.528 | 5020.79 |
| paired | 1024 | 523776 | 1536 | 512 | 2438.290 | 1.190 | 2048.98 |
| dense | 1024 | 523776 | 523776 | 523776 | 572.069 | 563.030 | 1.02 |

Базовый полный перебор вызывает `TrianglesIntersect` для всех пар и не фильтрует их по AABB.
Сравнение времени включает эффект дерева и дополнительной проверки боксов; их отдельный вклад здесь не измерялся.

На разнесённых сценах BVH исключает все подробные проверки треугольников.
В paired остаются подробные проверки совпадающих пар. В dense все боксы перекрываются:
число пар остаётся N(N-1)/2, а дерево добавляет расходы. BVH не гарантирует ускорение для любой сцены.
Эти искусственные сцены проверяют ожидаемые режимы работы; они не описывают все реальные нагрузки.

Времена зависят от машины и нагрузки WSL. Это короткие замеры, без доверительных интервалов.
Расход памяти и графический FPS здесь не измерялись.

Данные: [measurements.csv](measurements.csv); отдельные запуски: n128.csv, n512.csv, n1024.csv.

Повторение из корня проекта:

```bash
cmake -S . -B /tmp/triangle3d-release -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build /tmp/triangle3d-release --target bvh_benchmark -j2
/tmp/triangle3d-release/bvh_benchmark 128 3
/tmp/triangle3d-release/bvh_benchmark 512 3
/tmp/triangle3d-release/bvh_benchmark 1024 3
```

Проверка корректности: 457 тестов проходят в обычной сборке и с
AddressSanitizer/UndefinedBehaviorSanitizer; [журнал](verification.txt).
Во время проверки исправлено чтение за концом массива в суммах expansions
из `third_party/robust_predicates`: добавлены проверки границ, порядок
арифметических действий сохранён. Добавлены три регрессионных теста.
Проверка утечек отключена (`ASAN_OPTIONS=detect_leaks=0`); утечки этим запуском не проверялись.

Повторение проверки санитайзерами (используется уже скачанный GoogleTest):

```bash
cmake -S . -B /tmp/triangle3d-sanitized -DCMAKE_BUILD_TYPE=Debug \
  -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$PWD/build/_deps/googletest-src" \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer"
ASAN_OPTIONS=detect_leaks=0 cmake --build /tmp/triangle3d-sanitized --target geometry_tests input_tests -j2
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 /tmp/triangle3d-sanitized/geometry_tests --gtest_brief=1
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 /tmp/triangle3d-sanitized/input_tests --gtest_brief=1
```
