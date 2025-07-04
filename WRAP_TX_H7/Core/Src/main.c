/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "constants.h"
#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

OPAMP_HandleTypeDef hopamp1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

int symbol_buffer[NUM_SYMBS] = {0};

uint8_t parity_bits[4] = {0};
uint8_t parity_bit_index;

float buf1[DAC_BUF_LEN] = {0};
float buf2[DAC_BUF_LEN + RRC_LEN - 1] = {0};
uint32_t dac_buf[DAC_BUF_LEN + RRC_LEN - 1] = {3660 ,2545 ,750 ,754 ,2541 ,3636 ,2537 ,775 ,781 ,2529 ,3596 ,2523 ,811 ,820 ,2514 ,3545 ,2507 ,856 ,865 ,2496 ,3487 ,2489 ,903 ,913 ,2478 ,3428 ,2471 ,950 ,959 ,2461 ,3373 ,2454 ,992 ,999 ,2446 ,3327 ,2441 ,1025 ,1031 ,2435 ,3293 ,2431 ,1048 ,1051 ,2428 ,3274 ,2426 ,1059 ,1060 ,2425 ,3252 ,2420 ,1072 ,1070 ,2422 ,3263 ,2425 ,1058 ,1054 ,2430 ,3289 ,2434 ,1032 ,1026 ,2441 ,3328 ,2446 ,998 ,990 ,2455 ,3376 ,2461 ,957 ,949 ,2471 ,3428 ,2478 ,915 ,906 ,2487 ,3480 ,2494 ,874 ,866 ,2502 ,3527 ,2508 ,838 ,832 ,2515 ,3564 ,2518 ,812 ,808 ,2523 ,3589 ,2525 ,797 ,795 ,2527 ,3599 ,2527 ,794 ,795 ,2526 ,3592 ,2524 ,805 ,809 ,2520 ,3569 ,2516 ,828 ,834 ,2509 ,3532 ,2504 ,862 ,870 ,2495 ,3483 ,2488 ,904 ,913 ,2478 ,3428 ,2471 ,950 ,960 ,2460 ,3370 ,2453 ,997 ,1005 ,2443 ,3315 ,2437 ,1039 ,1046 ,2428 ,3269 ,2423 ,1072 ,1078 ,2417 ,3236 ,2414 ,1094 ,1097 ,2411 ,3202 ,2404 ,1116 ,1115 ,2405 ,3205 ,2407 ,1106 ,1102 ,2411 ,3231 ,2416 ,1078 ,1070 ,2425 ,3279 ,2432 ,1032 ,1021 ,2445 ,3346 ,2454 ,973 ,959 ,2469 ,3428 ,2480 ,903 ,888 ,2497 ,3518 ,2508 ,828 ,814 ,2525 ,3610 ,2536 ,756 ,742 ,2552 ,3694 ,2561 ,692 ,681 ,2574 ,3762 ,2581 ,645 ,638 ,2589 ,3805 ,2592 ,620 ,618 ,2594 ,3815 ,2593 ,624 ,628 ,2588 ,3786 ,2582 ,661 ,673 ,2568 ,3713 ,2556 ,735 ,754 ,2534 ,3594 ,2517 ,847 ,873 ,2486 ,3428 ,2462 ,995 ,1029 ,2424 ,3218 ,2395 ,1179 ,1219 ,2349 ,2969 ,2316 ,1391 ,1437 ,2264 ,2687 ,2227 ,1627 ,1677 ,2171 ,2383 ,2132 ,1879 ,1930 ,2073 ,2048 ,2023 ,2166 ,2217 ,1964 ,1713 ,1925 ,2419 ,2469 ,1869 ,1409 ,1832 ,2659 ,2705 ,1780 ,1127 ,1747 ,2877 ,2917 ,1701 ,878 ,1672 ,3067 ,3101 ,1634 ,668 ,1610 ,3223 ,3249 ,1579 ,502 ,1562 ,3342 ,3361 ,1540 ,383 ,1528 ,3423 ,3435 ,1514 ,310 ,1508 ,3468 ,3472 ,1503 ,281 ,1502 ,3478 ,3476 ,1504 ,291 ,1507 ,3458 ,3451 ,1515 ,334 ,1522 ,3415 ,3404 ,1535 ,402 ,1544 ,3354 ,3340 ,1560 ,486 ,1571 ,3282 ,3268 ,1588 ,578 ,1599 ,3208 ,3193 ,1616 ,668 ,1627 ,3137 ,3123 ,1642 ,750 ,1651 ,3075 ,3064 ,1664 ,817 ,1671 ,3026 ,3018 ,1680 ,865 ,1685 ,2994 ,2990 ,1689 ,891 ,1691 ,2981 ,2980 ,1692 ,912 ,1691 ,2985 ,2987 ,1688 ,878 ,1685 ,3004 ,3010 ,1678 ,843 ,1673 ,3038 ,3046 ,1663 ,792 ,1656 ,3083 ,3092 ,1645 ,732 ,1638 ,3133 ,3143 ,1626 ,668 ,1618 ,3185 ,3195 ,1606 ,605 ,1599 ,3234 ,3243 ,1588 ,550 ,1582 ,3275 ,3282 ,1574 ,507 ,1570 ,3306 ,3310 ,1564 ,479 ,1562 ,3322 ,3324 ,1560 ,470 ,1560 ,3324 ,3322 ,1562 ,479 ,1564 ,3310 ,3306 ,1570 ,507 ,1574 ,3282 ,3275 ,1582 ,550 ,1588 ,3243 ,3234 ,1599 ,605 ,1606 ,3195 ,3185 ,1618 ,668 ,1626 ,3143 ,3133 ,1638 ,732 ,1645 ,3092 ,3083 ,1656 ,792 ,1663 ,3046 ,3038 ,1673 ,843 ,1678 ,3010 ,3004 ,1685 ,878 ,1688 ,2987 ,2985 ,1691 ,877 ,1681 ,3009 ,3010 ,1680 ,856 ,1679 ,3018 ,3021 ,1675 ,834 ,1671 ,3042 ,3048 ,1663 ,794 ,1657 ,3080 ,3089 ,1647 ,737 ,1639 ,3130 ,3141 ,1626 ,668 ,1617 ,3189 ,3201 ,1603 ,592 ,1593 ,3251 ,3264 ,1579 ,514 ,1569 ,3313 ,3324 ,1556 ,443 ,1548 ,3367 ,3377 ,1537 ,384 ,1531 ,3408 ,3414 ,1524 ,347 ,1521 ,3431 ,3432 ,1519 ,337 ,1520 ,3428 ,3425 ,1524 ,360 ,1529 ,3398 ,3388 ,1541 ,421 ,1551 ,3335 ,3319 ,1570 ,524 ,1585 ,3239 ,3215 ,1612 ,668 ,1632 ,3108 ,3078 ,1667 ,854 ,1692 ,2945 ,2908 ,1734 ,1079 ,1764 ,2751 ,2710 ,1812 ,1337 ,1846 ,2533 ,2486 ,1899 ,1624 ,1935 ,2294 ,2244 ,1992 ,1912 ,2031 ,2042 ,1991 ,2089 ,2245 ,2129 ,1785 ,1734 ,2188 ,2563 ,2226 ,1530 ,1480 ,2284 ,2872 ,2321 ,1285 ,1238 ,2375 ,3163 ,2410 ,1057 ,1014 ,2459 ,3428 ,2490 ,854 ,816 ,2532 ,3657 ,2558 ,681 ,651 ,2593 ,3844 ,2613 ,545 ,523 ,2638 ,3982 ,2652 ,451 ,437 ,2668 ,4066 ,2675 ,400 ,396 ,2680 ,4095 ,2680 ,396 ,400 ,2675 ,4066 ,2668 ,437 ,451 ,2652 ,3982 ,2638 ,523 ,545 ,2613 ,3844 ,2593 ,651 ,681 ,2558 ,3657 ,2532 ,816 ,854 ,2490 ,3428 ,2459 ,1014 ,1057 ,2410 ,3163 ,2375 ,1238 ,1285 ,2321 ,2872 ,2284 ,1480 ,1530 ,2226 ,2563 ,2188 ,1734 ,1785 ,2129 ,2245 ,2089 ,1991 ,2042 ,2031 ,1948 ,2003 ,2215 ,2265 ,1946 ,1659 ,1909 ,2459 ,2506 ,1855 ,1369 ,1821 ,2686 ,2730 ,1772 ,1102 ,1740 ,2892 ,2931 ,1697 ,867 ,1670 ,3072 ,3104 ,1633 ,668 ,1611 ,3220 ,3245 ,1581 ,509 ,1564 ,3335 ,3354 ,1543 ,393 ,1531 ,3415 ,3427 ,1517 ,319 ,1511 ,3461 ,3467 ,1505 ,286 ,1503 ,3475 ,3474 ,1504 ,291 ,1507 ,3460 ,3454 ,1514 ,329 ,1520 ,3421 ,3410 ,1532 ,393 ,1541 ,3362 ,3348 ,1557 ,476 ,1568 ,3290 ,3275 ,1585 ,570 ,1597 ,3212 ,3196 ,1616 ,668 ,1628 ,3133 ,3118 ,1645 ,761 ,1656 ,3061 ,3048 ,1671 ,842 ,1680 ,3001 ,2991 ,1691 ,905 ,1698 ,2957 ,2951 ,1705 ,945 ,1709 ,2933 ,2931 ,1711 ,977 ,1722 ,2902 ,2904 ,1720 ,980 ,1716 ,2923 ,2930 ,1708 ,936 ,1701 ,2968 ,2979 ,1688 ,866 ,1677 ,3032 ,3047 ,1660 ,774 ,1648 ,3112 ,3129 ,1628 ,668 ,1615 ,3200 ,3219 ,1594 ,556 ,1580 ,3291 ,3308 ,1560 ,448 ,1547 ,3375 ,3391 ,1529 ,352 ,1519 ,3447 ,3459 ,1505 ,278 ,1498 ,3497 ,3504 ,1490 ,235 ,1487 ,3520 ,3521 ,1486 ,230 ,1488 ,3510 ,3504 ,1495 ,269 ,1502 ,3464 ,3450 ,1518 ,354 ,1532 ,3380 ,3358 ,1556 ,488 ,1576 ,3256 ,3227 ,1609 ,668 ,1634 ,3096 ,3060 ,1676 ,891 ,1706 ,2903 ,2861 ,1754 ,1151 ,1788 ,2683 ,2636 ,1842 ,1440 ,1879 ,2442 ,2392 ,1936 ,1748 ,1975 ,2188 ,2137 ,2034 ,2048 ,2062 ,1959 ,1908 ,2121 ,2348 ,2160 ,1704 ,1654 ,2217 ,2656 ,2254 ,1460 ,1413 ,2308 ,2945 ,2342 ,1235 ,1193 ,2390 ,3205 ,2420 ,1036 ,1000 ,2462 ,3428 ,2487 ,869 ,840 ,2520 ,3608 ,2540 ,738 ,716 ,2564 ,3742 ,2578 ,646 ,632 ,2594 ,3827 ,2601 ,592 ,586 ,2608 ,3866 ,2610 ,575 ,576 ,2609 ,3861 ,2606 ,592 ,599 ,2598 ,3818 ,2591 ,637 ,649 ,2577 ,3744 ,2567 ,705 ,721 ,2549 ,3648 ,2536 ,788 ,805 ,2516 ,3540 ,2502 ,877 ,896 ,2481 ,3428 ,2468 ,967 ,984 ,2448 ,3322 ,2436 ,1049 ,1064 ,2419 ,3230 ,2408 ,1117 ,1128 ,2395 ,3160 ,2388 ,1166 ,1173 ,2380 ,3116 ,2376 ,1192 ,1194 ,2374 ,3119 ,2385 ,1165 ,1163 ,2387 ,3151 ,2391 ,1145 ,1139 ,2398 ,3191 ,2405 ,1105 ,1095 ,2416 ,3254 ,2425 ,1048 ,1035 ,2440 ,3335 ,2451 ,978 ,963 ,2468 ,3428 ,2480 ,900 ,884 ,2499 ,3526 ,2511 ,821 ,806 ,2528 ,3620 ,2539 ,748 ,734 ,2555 ,3703 ,2564 ,686 ,675 ,2576 ,3767 ,2582 ,642 ,636 ,2589 ,3805 ,2592 ,622 ,621 ,2593 ,3810 ,2591 ,629 ,635 ,2585 ,3777 ,2579 ,669 ,681 ,2565 ,3703 ,2553 ,742 ,761 ,2532 ,3587 ,2515 ,851 ,876 ,2485 ,3428 ,2463 ,992 ,1024 ,2426 ,3229 ,2399 ,1165 ,1204 ,2356 ,2994 ,2324 ,1366 ,1410 ,2275 ,2727 ,2241 ,1590 ,1637 ,2187 ,2437 ,2150 ,1831 ,1881 ,2093 ,2113 ,2054 ,2083 ,2134 ,1996 ,1815 ,1957 ,2338 ,2389 ,1899 ,1502 ,1860 ,2589 ,2638 ,1804 ,1201 ,1768 ,2827 ,2872 ,1716 ,920 ,1683 ,3045 ,3086 ,1636 ,668 ,1607 ,3238 ,3273 ,1567 ,453 ,1543 ,3399 ,3427 ,1511 ,280 ,1493 ,3523 ,3544 ,1470 ,155 ,1457 ,3609 ,3621 ,1443 ,80 ,1437 ,3653 ,3656 ,1433 ,57 ,1433 ,3655 ,3650 ,1438 ,85 ,1445 ,3615 ,3602 ,1460 ,164 ,1473 ,3536 ,3515 ,1496 ,290 ,1514 ,3419 ,3392 ,1546 ,460 ,1569 ,3269 ,3235 ,1608 ,668 ,1636 ,3089 ,3050 ,1680 ,909 ,1712 ,2885 ,2842 ,1761 ,1176 ,1796 ,2663 ,2616 ,1849 ,1462 ,1885 ,2426 ,2378 ,1941 ,1761 ,1978 ,2182 ,2132 ,2035 ,2083 ,2072 ,1935 ,1885 ,2129 ,2370 ,2166 ,1691 ,1643 ,2221 ,2665 ,2256 ,1457 ,1412 ,2308 ,2944 ,2341 ,1238 ,1196 ,2389 ,3200 ,2419 ,1039 ,1002 ,2461 ,3428 ,2487 ,865 ,834 ,2523 ,3622 ,2545 ,721 ,695 ,2574 ,3777 ,2590 ,608 ,590 ,2612 ,3891 ,2623 ,530 ,519 ,2636 ,3960 ,2641 ,489 ,485 ,2646 ,3983 ,2646 ,485 ,489 ,2641 ,3960 ,2636 ,519 ,530 ,2623 ,3891 ,2612 ,590 ,608 ,2590 ,3777 ,2574 ,695 ,721 ,2545 ,3622 ,2523 ,834 ,865 ,2487 ,3428 ,2461 ,1002 ,1039 ,2419 ,3200 ,2389 ,1196 ,1238 ,2341 ,2944 ,2308 ,1412 ,1457 ,2256 ,2665 ,2221 ,1643 ,1691 ,2166 ,2370 ,2129 ,1885 ,1935 ,2072 ,2048 ,2024 ,2161 ,2211 ,1967 ,1726 ,1930 ,2405 ,2453 ,1875 ,1431 ,1840 ,2639 ,2684 ,1788 ,1152 ,1755 ,2858 ,2900 ,1707 ,896 ,1677 ,3057 ,3094 ,1635 ,668 ,1609 ,3231 ,3262 ,1573 ,474 ,1551 ,3375 ,3401 ,1522 ,319 ,1506 ,3488 ,3506 ,1484 ,205 ,1473 ,3566 ,3577 ,1460 ,136 ,1455 ,3607 ,3611 ,1450 ,113 ,1450 ,3611 ,3607 ,1455 ,136 ,1460 ,3577 ,3566 ,1473 ,205 ,1484 ,3506 ,3488 ,1506 ,319 ,1522 ,3401 ,3375 ,1551 ,474 ,1573 ,3262 ,3231 ,1609 ,668 ,1635 ,3094 ,3057 ,1677 ,896 ,1707 ,2900 ,2858 ,1755 ,1152 ,1788 ,2684 ,2639 ,1840 ,1431 ,1875 ,2453 ,2405 ,1930 ,1726 ,1967 ,2211 ,2161 ,2024 ,2048 ,2072 ,1935 ,1885 ,2129 ,2370 ,2166 ,1691 ,1643 ,2221 ,2665 ,2256 ,1457 ,1412 ,2308 ,2944 ,2341 ,1238 ,1196 ,2389 ,3200 ,2419 ,1039 ,1002 ,2461 ,3428 ,2487 ,865 ,834 ,2523 ,3622 ,2545 ,721 ,695 ,2574 ,3777 ,2590 ,608 ,590 ,2612 ,3891 ,2623 ,530 ,519 ,2636 ,3960 ,2641 ,489 ,485 ,2646 ,3983 ,2646 ,485 ,489 ,2641 ,3960 ,2636 ,519 ,530 ,2623 ,3891 ,2612 ,590 ,608 ,2590 ,3777 ,2574 ,695 ,721 ,2545 ,3622 ,2523 ,834 ,865 ,2487 ,3428 ,2461 ,1002 ,1039 ,2419 ,3200 ,2389 ,1196 ,1238 ,2341 ,2944 ,2308 ,1412 ,1457 ,2256 ,2665 ,2221 ,1643 ,1691 ,2166 ,2370 ,2129 ,1885 ,1935 ,2072 ,2083 ,2035 ,2132 ,2182 ,1978 ,1761 ,1941 ,2378 ,2426 ,1885 ,1462 ,1849 ,2616 ,2663 ,1796 ,1176 ,1761 ,2842 ,2885 ,1712 ,909 ,1680 ,3050 ,3089 ,1636 ,668 ,1608 ,3235 ,3269 ,1569 ,460 ,1546 ,3392 ,3419 ,1514 ,290 ,1496 ,3515 ,3536 ,1473 ,164 ,1460 ,3602 ,3615 ,1445 ,85 ,1438 ,3650 ,3655 ,1433 ,57 ,1433 ,3656 ,3653 ,1437 ,80 ,1443 ,3621 ,3609 ,1457 ,155 ,1470 ,3544 ,3523 ,1493 ,280 ,1511 ,3427 ,3399 ,1543 ,453 ,1567 ,3273 ,3238 ,1607 ,668 ,1636 ,3086 ,3045 ,1683 ,920 ,1716 ,2872 ,2827 ,1768 ,1201 ,1804 ,2638 ,2589 ,1860 ,1502 ,1899 ,2389 ,2338 ,1957 ,1815 ,1996 ,2134 ,2083 ,2054 ,2113 ,2093 ,1881 ,1831 ,2150 ,2437 ,2187 ,1637 ,1590 ,2241 ,2727 ,2275 ,1410 ,1366 ,2324 ,2994 ,2356 ,1204 ,1165 ,2399 ,3229 ,2426 ,1024 ,992 ,2463 ,3428 ,2485 ,876 ,851 ,2515 ,3587 ,2532 ,761 ,742 ,2553 ,3703 ,2565 ,681 ,669 ,2579 ,3777 ,2585 ,635 ,629 ,2591 ,3810 ,2593 ,621 ,622 ,2592 ,3805 ,2589 ,636 ,642 ,2582 ,3767 ,2576 ,675 ,686 ,2564 ,3703 ,2555 ,734 ,748 ,2539 ,3620 ,2528 ,806 ,821 ,2511 ,3526 ,2499 ,884 ,900 ,2480 ,3428 ,2468 ,963 ,978 ,2451 ,3335 ,2440 ,1035 ,1048 ,2425 ,3254 ,2416 ,1095 ,1105 ,2405 ,3191 ,2398 ,1139 ,1145 ,2391 ,3151 ,2387 ,1163 ,1165 ,2385 ,3154 ,2385 ,1165 ,1163 ,2387 ,3151 ,2391 ,1145 ,1139 ,2398 ,3191 ,2405 ,1105 ,1095 ,2416 ,3254 ,2425 ,1048 ,1035 ,2440 ,3335 ,2451 ,978 ,963 ,2468 ,3428 ,2480 ,900 ,884 ,2499 ,3526 ,2511 ,821 ,806 ,2528 ,3620 ,2539 ,748 ,734 ,2555 ,3703 ,2564 ,686 ,675 ,2576 ,3767 ,2582 ,642 ,636 ,2589 ,3805 ,2592 ,622 ,621 ,2593 ,3810 ,2591 ,629 ,635 ,2585 ,3777 ,2579 ,669 ,681 ,2565 ,3703 ,2553 ,742 ,761 ,2532 ,3587 ,2515 ,851 ,876 ,2485 ,3428 ,2463 ,992 ,1024 ,2426 ,3229 ,2399 ,1165 ,1204 ,2356 ,2994 ,2324 ,1366 ,1410 ,2275 ,2727 ,2241 ,1590 ,1637 ,2187 ,2437 ,2150 ,1831 ,1881 ,2093 ,2113 ,2054 ,2083 ,2134 ,1996 ,1815 ,1957 ,2338 ,2389 ,1899 ,1502 ,1860 ,2589 ,2638 ,1804 ,1201 ,1768 ,2827 ,2872 ,1716 ,920 ,1683 ,3045 ,3086 ,1636 ,668 ,1607 ,3238 ,3273 ,1567 ,453 ,1543 ,3399 ,3427 ,1511 ,280 ,1493 ,3523 ,3544 ,1470 ,155 ,1457 ,3609 ,3621 ,1443 ,80 ,1437 ,3653 ,3656 ,1433 ,57 ,1433 ,3655 ,3650 ,1438 ,85 ,1445 ,3615 ,3602 ,1460 ,164 ,1473 ,3536 ,3515 ,1496 ,290 ,1514 ,3419 ,3392 ,1546 ,460 ,1569 ,3269 ,3235 ,1608 ,668 ,1636 ,3089 ,3050 ,1680 ,909 ,1712 ,2885 ,2842 ,1761 ,1176 ,1796 ,2663 ,2616 ,1849 ,1462 ,1885 ,2426 ,2378 ,1941 ,1761 ,1978 ,2182 ,2132 ,2035 ,2048 ,2061 ,1964 ,1914 ,2118 ,2335 ,2155 ,1718 ,1670 ,2211 ,2634 ,2247 ,1480 ,1433 ,2300 ,2920 ,2335 ,1254 ,1211 ,2384 ,3187 ,2416 ,1046 ,1007 ,2460 ,3428 ,2488 ,861 ,827 ,2527 ,3636 ,2550 ,704 ,677 ,2582 ,3806 ,2600 ,581 ,560 ,2623 ,3932 ,2636 ,494 ,481 ,2651 ,4011 ,2658 ,446 ,441 ,2663 ,4039 ,2663 ,440 ,443 ,2659 ,4016 ,2653 ,475 ,487 ,2639 ,3941 ,2626 ,552 ,573 ,2603 ,3816 ,2585 ,669 ,697 ,2553 ,3643 ,2529 ,823 ,858 ,2489 ,3428 ,2460 ,1010 ,1051 ,2413 ,3176 ,2380 ,1224 ,1269 ,2328 ,2895 ,2292 ,1458 ,1507 ,2236 ,2594 ,2197 ,1707 ,1758 ,2139 ,2281 ,2100 ,1962 ,2013 ,2042 ,1948 ,1992 ,2244 ,2294 ,1935 ,1624 ,1899 ,2486 ,2533 ,1846 ,1337 ,1812 ,2710 ,2751 ,1764 ,1079 ,1734 ,2908 ,2945 ,1692 ,854 ,1667 ,3078 ,3108 ,1632 ,668 ,1612 ,3215 ,3239 ,1585 ,524 ,1570 ,3319 ,3335 ,1551 ,421 ,1541 ,3388 ,3398 ,1529 ,360 ,1524 ,3425 ,3428 ,1520 ,337 ,1519 ,3432 ,3431 ,1521 ,347 ,1524 ,3414 ,3408 ,1531 ,384 ,1537 ,3377 ,3367 ,1548 ,443 ,1556 ,3324 ,3313 ,1569 ,514 ,1579 ,3264 ,3251 ,1593 ,592 ,1603 ,3201 ,3189 ,1617 ,668 ,1626 ,3141 ,3130 ,1639 ,737 ,1647 ,3089 ,3080 ,1657 ,794 ,1663 ,3048 ,3042 ,1671 ,834 ,1675 ,3021 ,3018 ,1679 ,856 ,1680 ,3010 ,3009 ,1681 ,859 ,1685 ,2999 ,3002 ,1682 ,860 ,1679 ,3018 ,3024 ,1673 ,827 ,1668 ,3050 ,3057 ,1659 ,781 ,1653 ,3091 ,3099 ,1643 ,726 ,1636 ,3136 ,3146 ,1625 ,668 ,1618 ,3183 ,3192 ,1608 ,613 ,1601 ,3226 ,3234 ,1592 ,564 ,1587 ,3262 ,3268 ,1580 ,527 ,1576 ,3287 ,3291 ,1572 ,504 ,1570 ,3301 ,3302 ,1569 ,497 ,1569 ,3301 ,3299 ,1571 ,507 ,1573 ,3288 ,3284 ,1578 ,532 ,1581 ,3264 ,3258 ,1588 ,569 ,1594 ,3230 ,3222 ,1602 ,616 ,1609 ,3190 ,3181 ,1618 ,668 ,1625 ,3147 ,3139 ,1635 ,720 ,1641 ,3106 ,3098 ,1650 ,768 ,1655 ,3070 ,3064 ,1662 ,807 ,1666 ,3042 ,3038 ,1671 ,833 ,1674 ,3026 ,3024 ,1676 ,844 ,1671 ,3036 ,3037 ,1670 ,822 ,1668 ,3045 ,3048 ,1665 ,803 ,1661 ,3065 ,3071 ,1655 ,769 ,1650 ,3097 ,3104 ,1642 ,723 ,1635 ,3137 ,3146 ,1625 ,668 ,1618 ,3183 ,3193 ,1607 ,609 ,1600 ,3231 ,3240 ,1589 ,551 ,1582 ,3276 ,3285 ,1573 ,500 ,1567 ,3315 ,3321 ,1559 ,460 ,1555 ,3342 ,3346 ,1551 ,436 ,1549 ,3355 ,3356 ,1549 ,433 ,1550 ,3350 ,3347 ,1553 ,454 ,1558 ,3325 ,3318 ,1566 ,500 ,1573 ,3280 ,3268 ,1587 ,572 ,1597 ,3214 ,3198 ,1615 ,668 ,1628 ,3128 ,3109 ,1650 ,787 ,1666 ,3026 ,3003 ,1692 ,924 ,1710 ,2910 ,2885 ,1738 ,1074 ,1757 ,2785 ,2759 ,1786 ,1233 ,1806 ,2655 ,2629 ,1836 ,1394 ,1861 ,2511 ,2486 ,1890 ,1568 ,1909 ,2388 ,2364 ,1936 ,1714 ,1953 ,2274 ,2253 ,1977 ,1844 ,1993 ,2174 ,2156 ,2013 ,1957 ,2026 ,2090 ,2075 ,2043 ,2048 ,2053 ,2023 ,2012 ,2066 ,2117 ,2073 ,1974 ,1967 ,2082 ,2165 ,2086 ,1943 ,1938 ,2091 ,2192 ,2093 ,1927 ,1926 ,2095 ,2200 ,2095 ,1926 ,1927 ,2094};
//uint32_t dac_buf[DAC_BUF_LEN + RRC_LEN - 1] = {0};

char rx_msg[NUM_CHARS];
char temp_msg[NUM_CHARS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC1_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void recalculate_output(uint8_t * message, uint16_t len) {

	// Reset symbol buffer from last time
	for(int i = 0; i < NUM_SYMBS; i++) {
		symbol_buffer[i] = 0;
	}

	// Copy received message into temp so we don't change the original (for debugging)
	for (size_t i = 0; i < sizeof(rx_msg); ++i) {
		temp_msg[i] = message[i];
	}

	// put the packet header in the symbol buffer
	memcpy(symbol_buffer, packet_header, 4*PACKET_HEADER_LEN);

	// put the message in the symbol buffer
	#if HAMMING_CODE_FLAG
		for(int i = 0; i < len; i++) {
			parity_bits[0] = (bool)(temp_msg[i] & 0x80) ^ (bool)(temp_msg[i] & 0x40) ^ (bool)(temp_msg[i] & 0x10) ^ (bool)(temp_msg[i] & 0x08) ^ (bool)(temp_msg[i] & 0x02);
			parity_bits[1] = (bool)(temp_msg[i] & 0x80) ^ (bool)(temp_msg[i] & 0x20) ^ (bool)(temp_msg[i] & 0x10) ^ (bool)(temp_msg[i] & 0x04) ^ (bool)(temp_msg[i] & 0x02);
			parity_bits[2] = (bool)(temp_msg[i] & 0x40) ^ (bool)(temp_msg[i] & 0x20) ^ (bool)(temp_msg[i] & 0x10) ^ (bool)(temp_msg[i] & 0x01);
			parity_bits[3] = (bool)(temp_msg[i] & 0x08) ^ (bool)(temp_msg[i] & 0x04) ^ (bool)(temp_msg[i] & 0x02) ^ (bool)(temp_msg[i] & 0x01);

			parity_bit_index = 0;
			for(int j = 0; j < BITS_PER_CHAR; j++) {
				if(((j+1) & j) == 0){
					symbol_buffer[PACKET_HEADER_LEN + BITS_PER_CHAR*i + j] = 2 * parity_bits[parity_bit_index] - 1;
					parity_bit_index++;
				}
				else {
					symbol_buffer[PACKET_HEADER_LEN + BITS_PER_CHAR*i + j] = 2 * (int)(bool)(temp_msg[i] & 0x80) - 1;;
					temp_msg[i] = temp_msg[i] << 1;
				}
			}
		}

	#elif REPETITION_CODE_FLAG
		for(int i = 0; i < len; i++) {
			for(int j = 0; j < 8; j++) {
				for(int k = 0; k < REPETITION_INVERSE_CODERATE; k++){
					symbol_buffer[PACKET_HEADER_LEN + BITS_PER_CHAR - 1 + BITS_PER_CHAR*i - REPETITION_INVERSE_CODERATE*j - k] = 2 * (int)(temp_msg[i] & 0x01) - 1;
				}
				temp_msg[i] = temp_msg[i] >> 1;
			}
		}

	#else
		for(int i = 0; i < len; i++) {
			for(int j = 0; j < 8; j++) {
				symbol_buffer[PACKET_HEADER_LEN + 8*i + 8 - 1 - j] = 2 * (int)(temp_msg[i] & 0x01) - 1;
				temp_msg[i] = temp_msg[i] >> 1;
			}
		}

	#endif

    // upsample symbols
    for (int i = 0; i < DAC_BUF_LEN; i++) {
	    if (i % SPS == 0) {
		    buf1[i] = (symbol_buffer[i / SPS]);
	    }
	    else {
		    buf1[i] = 0;
	    }
    }

    // filter
    arm_conv_f32(buf1, DAC_BUF_LEN, RRC, RRC_LEN, buf2);

    // modulate
	for (int i = 0; i < DAC_BUF_LEN + RRC_LEN - 1; i++) {
		dac_buf[i] = 0x800 + 5.5 * 0x7FF*cos(2 * M_PI * FC / FS * i) *  buf2[i];
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_DAC1_Init();
  MX_OPAMP1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_OPAMP_Start(&hopamp1);
  HAL_TIM_Base_Start(&htim6);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_UART_Receive_DMA(&huart3, (uint8_t *)rx_msg, sizeof(rx_msg));
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_buf, DAC_BUF_LEN + RRC_LEN - 1, DAC_ALIGN_12B_R);

  recalculate_output("hello!", NUM_CHARS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC_CH;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 55;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// TODO: Call a function to generate the waveform corresponding to the new message!
	HAL_UART_Transmit(&huart3, (uint8_t *)rx_msg, NUM_CHARS, 1);
	recalculate_output((uint8_t *) rx_msg, NUM_CHARS);

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
