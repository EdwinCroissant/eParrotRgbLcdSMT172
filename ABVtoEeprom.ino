#include "Arduino.h"
#include <I2C.h>				//https://github.com/rambo/I2C
#include <RgbLcdKeyShieldI2C.h>	//https://github.com/EdwinCroissantArduinoLibraries/RgbLcdKeyShieldI2C
//The setup function is called once at startup of the sketch

const char msgSplash1[] PROGMEM = "ABV table write";
const char msgSplash2[] PROGMEM = "V 0.01    (c) EC";
const char msgStart1[] PROGMEM = "S transfers";
const char msgStart2[] PROGMEM = "L verifies";
const char msgOk[] PROGMEM = "OK";
const char msgFail[] PROGMEM = "Fail";
uint16_t VaporAddress;
char lineBuffer[20];
uint16_t Counter = 0;

/*----make instance of the lcd ----*/
RgbLcdKeyShieldI2C lcd;

/*------- Tables to be transferred---------*/
const static uint16_t LiquidABV[] PROGMEM = {
		 966, 963, 960, 958, 956, 953, 951, 949, 947, 940, 937, 936, 936, 934, 933, 932, 930, 929, 928, 927,
		 926, 925, 923, 922, 921, 919, 918, 917, 916, 915, 914, 914, 913, 912, 911, 910, 907, 905, 904, 903,
		 902, 901, 900, 899, 898, 897, 895, 894, 893, 892, 892, 891, 890, 888, 887, 886, 885, 885, 884, 883,
		 882, 881, 880, 879, 878, 877, 876, 874, 873, 872, 871, 870, 869, 868, 867, 867, 866, 865, 864, 863,
		 862, 861, 860, 859, 858, 857, 856, 855, 854, 853, 852, 851, 850, 849, 848, 847, 846, 845, 844, 844,
		 843, 842, 841, 840, 839, 838, 837, 836, 835, 834, 833, 832, 831, 830, 830, 829, 828, 827, 826, 825,
		 824, 823, 822, 821, 820, 819, 818, 817, 817, 816, 815, 814, 813, 812, 811, 810, 809, 808, 807, 806,
		 805, 804, 803, 802, 801, 800, 799, 799, 798, 797, 796, 795, 794, 793, 792, 791, 790, 789, 788, 787,
		 786, 785, 784, 783, 782, 781, 780, 779, 778, 777, 776, 775, 774, 774, 773, 772, 771, 770, 769, 768,
		 767, 766, 765, 764, 763, 762, 761, 760, 759, 758, 757, 756, 755, 755, 754, 753, 752, 751, 750, 749,
		 748, 747, 746, 745, 744, 743, 742, 741, 741, 740, 739, 738, 737, 736, 735, 734, 733, 732, 732, 731,
		 730, 729, 728, 727, 726, 725, 724, 723, 722, 721, 721, 720, 719, 718, 717, 716, 715, 714, 713, 712,
		 711, 710, 710, 709, 708, 707, 706, 705, 705, 704, 704, 703, 703, 702, 702, 701, 701, 700, 700, 699,
		 698, 697, 696, 696, 695, 694, 693, 692, 692, 691, 690, 689, 688, 687, 686, 685, 684, 683, 682, 681,
		 680, 679, 677, 676, 675, 674, 673, 671, 670, 669, 668, 667, 666, 665, 664, 663, 662, 661, 660, 659,
		 658, 657, 656, 655, 654, 653, 652, 651, 650, 649, 649, 648, 647, 646, 645, 644, 643, 642, 641, 640,
		 639, 638, 637, 636, 635, 635, 634, 633, 632, 631, 630, 629, 628, 627, 626, 626, 625, 624, 623, 622,
		 621, 620, 619, 618, 617, 616, 615, 614, 613, 612, 611, 610, 609, 608, 607, 607, 606, 605, 604, 603,
		 602, 601, 600, 599, 598, 597, 596, 595, 594, 593, 592, 591, 590, 589, 588, 587, 586, 585, 584, 583,
		 582, 581, 581, 580, 579, 578, 577, 576, 575, 574, 573, 572, 571, 570, 570, 569, 568, 567, 566, 565,
		 564, 564, 563, 562, 561, 560, 559, 558, 557, 556, 555, 554, 553, 552, 552, 551, 550, 550, 549, 548,
		 547, 546, 545, 544, 543, 542, 541, 540, 539, 539, 538, 537, 536, 535, 534, 533, 532, 532, 531, 530,
		 529, 528, 527, 526, 525, 525, 524, 523, 522, 521, 520, 519, 518, 518, 517, 516, 515, 514, 513, 512,
		 511, 510, 509, 508, 507, 506, 505, 504, 503, 503, 502, 501, 500, 499, 498, 497, 496, 495, 495, 494,
		 493, 492, 491, 490, 489, 488, 487, 487, 486, 485, 484, 483, 482, 481, 480, 479, 478, 477, 476, 475,
		 474, 473, 472, 471, 470, 469, 469, 468, 467, 466, 465, 464, 463, 462, 461, 461, 460, 459, 458, 458,
		 457, 456, 455, 454, 454, 453, 452, 451, 451, 450, 449, 448, 447, 447, 446, 445, 444, 444, 443, 442,
		 441, 440, 440, 439, 438, 437, 436, 436, 435, 434, 433, 433, 432, 431, 430, 429, 428, 428, 427, 426,
		 425, 425, 424, 423, 422, 422, 421, 420, 420, 419, 418, 418, 417, 417, 416, 415, 415, 414, 414, 413,
		 412, 411, 411, 410, 409, 409, 408, 407, 406, 406, 405, 404, 404, 403, 402, 402, 401, 400, 399, 399,
		 398, 397, 397, 396, 395, 394, 394, 393, 392, 392, 391, 390, 390, 389, 388, 388, 387, 387, 386, 385,
		 385, 384, 384, 383, 382, 382, 381, 380, 380, 379, 378, 377, 377, 376, 375, 375, 374, 373, 372, 372,
		 371, 370, 370, 369, 368, 367, 367, 366, 365, 365, 364, 363, 363, 362, 361, 361, 360, 359, 359, 358,
		 357, 357, 356, 355, 354, 353, 353, 352, 351, 350, 350, 349, 348, 348, 347, 346, 346, 345, 345, 344,
		 343, 343, 342, 341, 341, 340, 340, 339, 338, 338, 337, 336, 336, 335, 334, 334, 333, 332, 332, 331,
		 330, 330, 329, 329, 328, 327, 327, 326, 326, 325, 324, 324, 323, 323, 322, 321, 321, 320, 320, 319,
		 318, 318, 317, 316, 316, 315, 314, 314, 313, 312, 312, 311, 311, 310, 309, 308, 308, 307, 306, 305,
		 305, 304, 303, 303, 302, 301, 301, 300, 299, 299, 298, 297, 297, 296, 296, 295, 294, 294, 293, 292,
		 292, 291, 290, 290, 289, 288, 288, 287, 286, 286, 285, 284, 284, 283, 282, 282, 281, 280, 280, 279,
		 278, 277, 277, 276, 275, 274, 274, 273, 272, 272, 271, 270, 270, 269, 269, 268, 268, 268, 267, 267,
		 267, 267, 266, 266, 266, 266, 265, 265, 265, 265, 264, 264, 264, 264, 264, 264, 264, 264, 263, 263,
		 263, 263, 263, 263, 263, 263, 263, 262, 262, 262, 262, 262, 261, 261, 261, 261, 261, 260, 260, 260,
		 260, 260, 260, 259, 259, 259, 259, 259, 258, 258, 258, 258, 258, 257, 257, 257, 257, 257, 256, 256,
		 256, 256, 256, 255, 255, 255, 255, 255, 255, 254, 254, 254, 254, 254, 253, 253, 253, 253, 253, 252,
		 252, 252, 252, 252, 251, 251, 251, 251, 251, 251, 251, 250, 250, 250, 250, 250, 250, 249, 249, 249,
		 249, 249, 248, 248, 248, 248, 247, 247, 247, 247, 247, 246, 246, 246, 246, 245, 245, 244, 244, 244,
		 243, 243, 242, 242, 242, 241, 241, 240, 240, 240, 239, 239, 238, 238, 238, 237, 237, 236, 236, 236,
		 235, 235, 234, 234, 234, 233, 233, 232, 232, 232, 231, 231, 230, 230, 230, 229, 229, 228, 228, 228,
		 227, 227, 226, 226, 225, 225, 225, 224, 224, 223, 223, 223, 222, 222, 222, 221, 221, 220, 220, 220,
		 219, 219, 219, 219, 218, 218, 218, 217, 217, 217, 216, 216, 216, 216, 215, 215, 215, 214, 214, 214,
		 213, 213, 213, 213, 212, 212, 212, 211, 211, 211, 211, 210, 210, 210, 210, 209, 209, 209, 208, 208,
		 208, 207, 207, 207, 207, 206, 206, 206, 205, 205, 205, 204, 204, 204, 204, 203, 203, 203, 202, 202,
		 202, 201, 201, 201, 200, 200, 200, 200, 199, 199, 199, 198, 198, 198, 197, 197, 197, 197, 196, 196,
		 196, 195, 195, 195, 195, 194, 194, 194, 194, 193, 193, 193, 192, 192, 192, 192, 191, 191, 191, 191,
		 190, 190, 190, 190, 189, 189, 189, 188, 188, 188, 188, 187, 187, 187, 187, 186, 186, 186, 185, 185,
		 185, 184, 184, 184, 184, 183, 183, 183, 182, 182, 182, 181, 181, 181, 180, 180, 180, 180, 179, 179,
		 179, 178, 178, 178, 177, 177, 177, 176, 176, 176, 176, 175, 175, 175, 174, 174, 174, 173, 173, 173,
		 173, 172, 172, 172, 172, 171, 171, 171, 170, 170, 170, 170, 169, 169, 169, 169, 168, 168, 168, 168,
		 167, 167, 167, 167, 166, 166, 166, 165, 165, 165, 165, 164, 164, 164, 164, 163, 163, 163, 163, 162,
		 162, 162, 162, 161, 161, 161, 161, 160, 160, 160, 160, 159, 159, 159, 159, 159, 158, 158, 158, 158,
		 158, 157, 157, 157, 157, 156, 156, 156, 156, 156, 155, 155, 155, 155, 154, 154, 154, 154, 153, 153,
		 153, 153, 152, 152, 152, 152, 151, 151, 151, 151, 150, 150, 150, 150, 149, 149, 149, 149, 148, 148,
		 148, 148, 148, 147, 147, 147, 147, 147, 146, 146, 146, 146, 146, 145, 145, 145, 145, 145, 144, 144,
		 144, 144, 144, 143, 143, 143, 143, 143, 142, 142, 142, 142, 141, 141, 141, 141, 141, 140, 140, 140,
		 140, 139, 139, 139, 139, 139, 138, 138, 138, 138, 137, 137, 137, 137, 137, 136, 136, 136, 136, 136,
		 135, 135, 135, 135, 135, 135, 134, 134, 134, 134, 134, 133, 133, 133, 133, 133, 133, 132, 132, 132,
		 132, 132, 132, 131, 131, 131, 131, 131, 131, 130, 130, 130, 130, 130, 129, 129, 129, 129, 129, 129,
		 128, 128, 128, 128, 128, 128, 127, 127, 127, 127, 127, 126, 126, 126, 126, 126, 126, 125, 125, 125,
		 125, 125, 124, 124, 124, 124, 124, 123, 123, 123, 123, 123, 122, 122, 122, 122, 122, 121, 121, 121,
		 121, 120, 120, 120, 120, 120, 119, 119, 119, 119, 119, 118, 118, 118, 118, 118, 117, 117, 117, 117,
		 117, 116, 116, 116, 116, 116, 115, 115, 115, 115, 115, 115, 114, 114, 114, 114, 114, 113, 113, 113,
		 113, 113, 112, 112, 112, 112, 112, 112, 111, 111, 111, 111, 111, 111, 110, 110, 110, 110, 110, 110,
		 109, 109, 109, 109, 109, 109, 108, 108, 108, 108, 108, 108, 107, 107, 107, 107, 107, 107, 106, 106,
		 106, 106, 106, 105, 105, 105, 105, 105, 105, 104, 104, 104, 104, 104, 103, 103, 103, 103, 103, 102,
		 102, 102, 102, 102, 101, 101, 101, 101, 101, 100, 100, 100, 100, 100,  99,  99,  99,  99,  99,  98,
		  98,  98,  98,  98,  97,  97,  97,  97,  96,  96,  96,  96,  96,  95,  95,  95,  95,  95,  94,  94,
		  94,  94,  94,  93,  93,  93,  93,  93,  92,  92,  92,  92,  92,  91,  91,  91,  91,  91,  91,  90,
		  90,  90,  90,  90,  89,  89,  89,  89,  89,  89,  88,  88,  88,  88,  88,  88,  87,  87,  87,  87,
		  87,  86,  86,  86,  86,  86,  86,  85,  85,  85,  85,  85,  85,  84,  84,  84,  84,  84,  84,  83,
		  83,  83,  83,  83,  83,  82,  82,  82,  82,  82,  82,  81,  81,  81,  81,  81,  81,  81,  81,  80,
		  80,  80,  80,  80,  80,  80,  80,  79,  79,  79,  79,  79,  79,  79,  79,  78,  78,  78,  78,  78,
		  78,  78,  77,  77,  77,  77,  77,  77,  77,  76,  76,  76,  76,  76,  76,  76,  75,  75,  75,  75,
		  75,  74,  74,  74,  74,  74,  73,  73,  73,  73,  72,  72,  72,  72,  72,  71,  71,  71,  71,  70,
		  70,  70,  70,  70,  69,  69,  69,  69,  69,  68,  68,  68,  68,  68,  67,  67,  67,  67,  67,  67,
		  66,  66,  66,  66,  66,  66,  65,  65,  65,  65,  65,  65,  64,  64,  64,  64,  64,  64,  63,  63,
		  63,  63,  63,  63,  63,  62,  62,  62,  62,  62,  62,  62,  61,  61,  61,  61,  61,  61,  61,  60,
		  60,  60,  60,  60,  60,  59,  59,  59,  59,  59,  59,  59,  58,  58,  58,  58,  58,  58,  58,  57,
		  57,  57,  57,  57,  57,  57,  56,  56,  56,  56,  56,  55,  55,  55,  55,  55,  55,  54,  54,  54,
		  54,  54,  54,  53,  53,  53,  53,  53,  53,  52,  52,  52,  52,  52,  52,  51,  51,  51,  51,  51,
		  51,  50,  50,  50,  50,  50,  50,  50,  49,  49,  49,  49,  49,  49,  49,  49,  48,  48,  48,  48,
		  48,  48,  48,  48,  48,  47,  47,  47,  47,  47,  47,  47,  47,  47,  46,  46,  46,  46,  46,  46,
		  46,  46,  46,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  44,  44,  44,  44,  44,  44,  44,
		  44,  44,  44,  44,  43,  43,  43,  43,  43,  43,  43,  43,  43,  42,  42,  42,  42,  42,  42,  42,
		  42,  42,  41,  41,  41,  41,  41,  41,  41,  41,  41,  40,  40,  40,  40,  40,  40,  40,  40,  40,
		  39,  39,  39,  39,  39,  39,  39,  39,  38,  38,  38,  38,  38,  38,  38,  37,  37,  37,  37,  37,
		  37,  37,  36,  36,  36,  36,  36,  36,  35,  35,  35,  35,  35,  35,  34,  34,  34,  34,  34,  34,
		  33,  33,  33,  33,  33,  33,  32,  32,  32,  32,  32,  32,  32,  31,  31,  31,  31,  31,  31,  31,
		  31,  30,  30,  30,  30,  30,  30,  30,  30,  29,  29,  29,  29,  29,  29,  29,  29,  28,  28,  28,
		  28,  28,  28,  28,  28,  28,  27,  27,  27,  27,  27,  27,  27,  27,  27,  26,  26,  26,  26,  26,
		  26,  26,  26,  26,  26,  25,  25,  25,  25,  25,  25,  25,  25,  25,  24,  24,  24,  24,  24,  24,
		  24,  24,  23,  23,  23,  23,  23,  23,  23,  23,  22,  22,  22,  22,  22,  22,  22,  22,  21,  21,
		  21,  21,  21,  21,  21,  21,  20,  20,  20,  20,  20,  20,  20,  19,  19,  19,  19,  19,  19,  19,
		  18,  18,  18,  18,  18,  18,  17,  17,  17,  17,  17,  17,  16,  16,  16,  16,  16,  16,  16,  15,
		  15,  15,  15,  15,  15,  14,  14,  14,  14,  14,  14,  14,  13,  13,  13,  13,  13,  13,  12,  12,
		  12,  12,  12,  12,  12,  12,  11,  11,  11,  11,  11,  11,  11,  11,  11,  10,  10,  10,  10,  10,
		  10,  10,  10,  10,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   8,   8,   8,   8,   8,
		   8,   8,   8,   8,   8,   8,   8,   8,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,
		   7,   7,   7,   7,   7,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,
		   6,   5,   5,   5,   5,   5,   4,   4,   4,   4,   3,   3,   3,   3,   2,   2,   2,   2,   1,   1,
		   1,   1,   0
};

const static uint16_t VaporABV[] PROGMEM = {
	    967, 964, 961, 960, 958, 956, 954, 953, 951, 946, 944, 944, 944, 943, 942, 941, 940, 939, 939, 938,
	    937, 937, 936, 935, 934, 934, 933, 932, 932, 931, 930, 930, 930, 930, 929, 928, 927, 925, 925, 924,
	    924, 923, 922, 922, 921, 920, 920, 919, 919, 918, 918, 918, 917, 916, 916, 915, 915, 915, 914, 914,
	    913, 913, 912, 912, 911, 911, 910, 910, 910, 909, 909, 908, 908, 907, 907, 906, 906, 905, 905, 905,
	    904, 904, 904, 903, 903, 902, 902, 901, 901, 900, 900, 900, 899, 899, 898, 898, 898, 897, 897, 897,
	    896, 896, 896, 896, 896, 896, 895, 895, 894, 894, 894, 893, 893, 893, 892, 892, 892, 891, 891, 891,
	    890, 890, 889, 889, 889, 888, 888, 888, 888, 888, 887, 887, 887, 887, 886, 886, 885, 885, 885, 884,
	    884, 884, 883, 883, 883, 882, 882, 882, 882, 882, 881, 881, 881, 880, 880, 879, 879, 879, 878, 878,
	    878, 877, 877, 877, 877, 876, 876, 876, 876, 876, 875, 875, 874, 874, 874, 873, 873, 873, 873, 872,
	    872, 872, 872, 871, 871, 871, 870, 870, 870, 870, 870, 869, 869, 869, 869, 869, 869, 868, 868, 868,
	    867, 867, 867, 867, 866, 866, 866, 866, 866, 865, 865, 865, 864, 864, 864, 864, 864, 864, 863, 863,
	    863, 862, 862, 862, 862, 861, 861, 861, 861, 860, 860, 860, 860, 859, 859, 859, 859, 859, 859, 858,
	    858, 858, 858, 858, 858, 857, 857, 857, 857, 857, 857, 857, 856, 856, 856, 856, 856, 855, 855, 855,
	    855, 854, 854, 854, 854, 854, 853, 853, 853, 853, 853, 853, 853, 852, 852, 852, 852, 852, 852, 851,
	    851, 850, 850, 850, 849, 849, 849, 849, 849, 848, 848, 848, 848, 848, 848, 847, 847, 847, 846, 846,
	    846, 846, 845, 845, 845, 845, 845, 845, 845, 844, 844, 844, 844, 844, 844, 843, 843, 843, 842, 842,
	    842, 842, 841, 841, 841, 841, 841, 841, 840, 840, 840, 840, 840, 840, 840, 839, 839, 839, 839, 839,
	    839, 838, 838, 838, 838, 838, 838, 838, 837, 837, 837, 837, 837, 837, 836, 836, 836, 836, 836, 835,
	    835, 835, 835, 834, 834, 834, 834, 833, 833, 833, 833, 833, 833, 833, 832, 832, 832, 832, 832, 831,
	    831, 831, 831, 831, 831, 831, 830, 830, 830, 830, 830, 830, 829, 829, 829, 829, 829, 829, 829, 829,
	    828, 828, 828, 828, 827, 827, 827, 827, 826, 826, 826, 826, 826, 826, 826, 826, 825, 825, 825, 825,
	    825, 824, 824, 824, 823, 823, 823, 823, 823, 822, 822, 822, 822, 822, 822, 822, 822, 821, 821, 821,
	    821, 821, 821, 820, 820, 820, 820, 819, 819, 819, 819, 818, 818, 818, 818, 818, 818, 818, 818, 818,
	    818, 818, 817, 817, 817, 816, 816, 816, 816, 815, 815, 815, 815, 814, 814, 814, 814, 814, 814, 814,
	    813, 813, 813, 813, 813, 812, 812, 812, 812, 811, 811, 811, 811, 811, 811, 810, 810, 810, 810, 810,
	    810, 809, 809, 809, 809, 809, 809, 808, 808, 808, 808, 807, 807, 807, 807, 807, 807, 806, 806, 806,
	    806, 806, 805, 805, 805, 805, 804, 804, 804, 804, 803, 803, 803, 803, 803, 802, 802, 802, 802, 802,
	    802, 802, 802, 802, 801, 801, 801, 801, 801, 801, 800, 800, 800, 799, 799, 799, 799, 798, 798, 798,
	    798, 798, 797, 797, 797, 797, 797, 797, 797, 797, 796, 796, 796, 796, 796, 795, 795, 795, 795, 795,
	    795, 795, 794, 794, 794, 794, 794, 794, 794, 794, 793, 793, 793, 793, 793, 793, 792, 792, 792, 791,
	    791, 791, 791, 790, 790, 790, 790, 789, 789, 789, 789, 789, 788, 788, 788, 788, 788, 787, 787, 787,
	    787, 787, 787, 787, 787, 787, 786, 786, 786, 786, 786, 786, 785, 785, 785, 785, 784, 784, 784, 784,
	    783, 783, 783, 783, 783, 782, 782, 782, 782, 782, 781, 781, 781, 781, 781, 780, 780, 780, 780, 780,
	    779, 779, 779, 779, 778, 778, 778, 778, 777, 777, 777, 777, 777, 776, 776, 776, 776, 776, 775, 775,
	    775, 774, 774, 774, 774, 773, 773, 773, 773, 772, 772, 772, 772, 772, 771, 771, 771, 771, 771, 770,
	    770, 770, 770, 770, 769, 769, 769, 769, 768, 768, 768, 768, 767, 767, 767, 766, 766, 766, 766, 765,
	    765, 765, 764, 764, 764, 764, 764, 763, 763, 763, 763, 763, 762, 762, 762, 761, 761, 761, 760, 760,
	    760, 759, 759, 759, 759, 759, 758, 758, 758, 758, 758, 757, 757, 757, 756, 756, 756, 756, 755, 755,
	    755, 754, 754, 754, 753, 753, 753, 752, 752, 752, 752, 751, 751, 751, 750, 750, 750, 749, 749, 749,
	    748, 748, 748, 747, 747, 747, 746, 746, 746, 746, 745, 745, 745, 744, 744, 744, 744, 744, 743, 743,
	    743, 743, 743, 743, 743, 742, 742, 742, 742, 742, 742, 742, 742, 741, 741, 741, 741, 741, 741, 741,
	    741, 741, 741, 741, 741, 740, 740, 740, 740, 740, 740, 740, 740, 739, 739, 739, 739, 739, 739, 739,
	    739, 739, 738, 738, 738, 738, 738, 738, 738, 738, 737, 737, 737, 737, 737, 737, 737, 737, 737, 736,
	    736, 736, 736, 736, 736, 736, 736, 735, 735, 735, 735, 735, 735, 735, 735, 734, 734, 734, 734, 734,
	    734, 734, 734, 734, 733, 733, 733, 733, 733, 733, 733, 733, 733, 733, 732, 732, 732, 732, 732, 732,
	    732, 732, 731, 731, 731, 731, 731, 731, 731, 730, 730, 730, 730, 730, 730, 729, 729, 729, 729, 729,
	    728, 728, 728, 728, 727, 727, 727, 727, 726, 726, 726, 726, 725, 725, 725, 725, 724, 724, 724, 724,
	    723, 723, 723, 723, 722, 722, 722, 721, 721, 721, 720, 720, 720, 720, 719, 719, 719, 718, 718, 718,
	    718, 717, 717, 717, 717, 716, 716, 716, 716, 715, 715, 715, 715, 714, 714, 714, 714, 713, 713, 713,
	    713, 712, 712, 712, 712, 711, 711, 711, 711, 710, 710, 710, 710, 709, 709, 709, 709, 709, 708, 708,
	    708, 708, 707, 707, 707, 707, 707, 706, 706, 706, 706, 705, 705, 705, 705, 705, 704, 704, 704, 704,
	    703, 703, 703, 703, 702, 702, 702, 702, 702, 701, 701, 701, 701, 700, 700, 700, 700, 699, 699, 699,
	    699, 698, 698, 698, 698, 697, 697, 697, 697, 697, 696, 696, 696, 696, 695, 695, 695, 695, 694, 694,
	    694, 694, 693, 693, 693, 693, 692, 692, 692, 692, 691, 691, 691, 690, 690, 690, 690, 689, 689, 689,
	    689, 688, 688, 688, 688, 687, 687, 687, 687, 686, 686, 686, 686, 685, 685, 685, 685, 684, 684, 684,
	    683, 683, 683, 683, 682, 682, 682, 681, 681, 681, 681, 680, 680, 680, 679, 679, 679, 679, 678, 678,
	    678, 677, 677, 677, 677, 676, 676, 676, 675, 675, 675, 675, 674, 674, 674, 673, 673, 673, 673, 672,
	    672, 672, 671, 671, 671, 671, 670, 670, 670, 669, 669, 669, 669, 668, 668, 668, 667, 667, 667, 666,
	    666, 666, 666, 665, 665, 665, 664, 664, 664, 664, 663, 663, 663, 662, 662, 662, 661, 661, 661, 661,
	    660, 660, 660, 659, 659, 659, 659, 658, 658, 658, 657, 657, 657, 657, 656, 656, 656, 656, 655, 655,
	    655, 654, 654, 654, 654, 653, 653, 653, 653, 652, 652, 652, 651, 651, 651, 650, 650, 650, 649, 649,
	    649, 649, 648, 648, 648, 647, 647, 647, 646, 646, 646, 645, 645, 645, 644, 644, 644, 643, 643, 643,
	    642, 642, 642, 642, 641, 641, 641, 640, 640, 640, 640, 639, 639, 639, 638, 638, 638, 637, 637, 637,
	    637, 636, 636, 636, 635, 635, 635, 634, 634, 634, 633, 633, 633, 632, 632, 632, 631, 631, 631, 630,
	    630, 630, 629, 629, 629, 628, 628, 627, 627, 627, 626, 626, 626, 625, 625, 625, 624, 624, 624, 623,
	    623, 623, 623, 622, 622, 622, 621, 621, 621, 620, 620, 620, 620, 619, 619, 619, 618, 618, 618, 617,
	    617, 617, 617, 616, 616, 616, 616, 615, 615, 615, 615, 614, 614, 614, 614, 613, 613, 613, 613, 612,
	    612, 612, 612, 611, 611, 611, 610, 610, 610, 610, 609, 609, 609, 609, 608, 608, 608, 607, 607, 607,
	    607, 606, 606, 606, 605, 605, 605, 604, 604, 604, 603, 603, 603, 602, 602, 601, 601, 601, 600, 600,
	    600, 599, 599, 598, 598, 598, 597, 597, 597, 596, 596, 595, 595, 595, 594, 594, 594, 593, 593, 593,
	    592, 592, 591, 591, 591, 590, 590, 590, 589, 589, 589, 588, 588, 588, 587, 587, 587, 586, 586, 586,
	    585, 585, 585, 584, 584, 584, 583, 583, 583, 582, 582, 582, 581, 581, 581, 580, 580, 580, 579, 579,
	    579, 578, 578, 578, 577, 577, 577, 576, 576, 576, 575, 575, 574, 574, 574, 573, 573, 573, 572, 572,
	    572, 571, 571, 571, 570, 570, 570, 569, 569, 568, 568, 568, 567, 567, 567, 566, 566, 565, 565, 565,
	    564, 564, 563, 563, 563, 562, 562, 561, 561, 561, 560, 560, 559, 559, 558, 558, 558, 557, 557, 556,
	    556, 555, 555, 554, 554, 553, 553, 552, 552, 551, 551, 550, 550, 549, 549, 548, 548, 547, 547, 546,
	    546, 545, 545, 544, 544, 544, 543, 543, 542, 542, 541, 541, 540, 540, 540, 539, 539, 538, 538, 537,
	    537, 536, 536, 536, 535, 535, 534, 534, 533, 533, 533, 532, 532, 531, 531, 530, 530, 529, 529, 529,
	    528, 528, 527, 527, 526, 526, 525, 525, 524, 524, 524, 523, 523, 522, 522, 521, 521, 520, 520, 519,
	    519, 518, 518, 518, 517, 517, 516, 516, 515, 515, 514, 514, 514, 513, 513, 512, 512, 512, 511, 511,
	    511, 510, 510, 510, 509, 509, 508, 508, 508, 507, 507, 506, 506, 506, 505, 505, 504, 504, 504, 503,
	    503, 502, 502, 501, 501, 500, 500, 500, 499, 499, 498, 498, 497, 497, 496, 496, 495, 495, 494, 494,
	    493, 492, 491, 491, 490, 489, 489, 488, 487, 487, 486, 485, 484, 484, 483, 482, 482, 481, 480, 479,
	    479, 478, 477, 476, 475, 475, 474, 473, 472, 472, 471, 470, 469, 469, 468, 467, 466, 466, 465, 464,
	    464, 463, 462, 461, 461, 460, 459, 459, 458, 457, 456, 456, 455, 454, 454, 453, 452, 451, 451, 450,
	    449, 449, 448, 447, 447, 446, 446, 445, 445, 444, 444, 443, 442, 442, 441, 441, 440, 440, 439, 439,
	    438, 438, 437, 436, 436, 435, 435, 434, 434, 433, 432, 432, 431, 431, 430, 429, 429, 428, 428, 427,
	    427, 426, 425, 425, 424, 424, 423, 422, 421, 420, 419, 418, 417, 416, 416, 415, 414, 413, 412, 411,
	    410, 409, 409, 408, 407, 406, 405, 404, 403, 402, 402, 401, 400, 399, 398, 397, 396, 395, 394, 394,
	    393, 392, 391, 390, 390, 389, 389, 388, 387, 387, 386, 386, 385, 385, 384, 384, 383, 382, 382, 381,
	    381, 380, 380, 379, 379, 378, 378, 377, 377, 376, 375, 375, 374, 374, 373, 373, 372, 372, 371, 371,
	    370, 370, 369, 369, 368, 368, 367, 367, 366, 366, 365, 365, 364, 364, 363, 363, 362, 362, 361, 361,
	    360, 360, 359, 358, 358, 357, 356, 356, 355, 354, 354, 353, 352, 352, 351, 350, 349, 349, 348, 347,
	    347, 346, 345, 344, 344, 343, 342, 341, 341, 340, 339, 338, 337, 337, 336, 335, 334, 333, 333, 332,
	    331, 330, 329, 328, 328, 327, 326, 325, 324, 323, 322, 322, 321, 320, 319, 318, 317, 316, 315, 314,
	    313, 312, 311, 310, 309, 308, 307, 306, 305, 304, 302, 301, 300, 299, 298, 297, 296, 295, 294, 293,
	    292, 291, 290, 289, 288, 287, 286, 285, 284, 283, 282, 281, 280, 279, 278, 278, 277, 276, 275, 274,
	    274, 273, 272, 271, 271, 270, 269, 268, 267, 267, 266, 265, 264, 263, 263, 262, 261, 260, 259, 259,
	    258, 257, 256, 255, 255, 254, 253, 252, 251, 251, 250, 249, 248, 247, 247, 246, 245, 244, 243, 242,
	    242, 241, 240, 239, 238, 238, 237, 236, 235, 234, 233, 232, 231, 230, 228, 227, 226, 225, 224, 223,
	    222, 221, 220, 219, 217, 216, 215, 214, 213, 212, 211, 209, 208, 207, 206, 205, 204, 202, 201, 200,
	    199, 198, 197, 195, 194, 193, 192, 190, 189, 188, 187, 186, 184, 183, 182, 181, 179, 178, 177, 175,
	    174, 173, 172, 170, 169, 168, 166, 165, 164, 163, 161, 160, 159, 158, 156, 155, 154, 153, 151, 150,
	    149, 148, 146, 145, 144, 143, 141, 140, 139, 138, 137, 135, 134, 133, 132, 131, 129, 128, 127, 126,
	    125, 124, 123, 122, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 109, 108, 107, 106, 105,
	    104, 103, 102, 101, 100, 100,  99,  98,  97,  96,  95,  94,  93,  92,  91,  91,  90,  89,  88,  87,
	     86,  86,  85,  84,  83,  82,  81,  81,  80,  79,  78,  78,  77,  76,  75,  75,  74,  73,  73,  72,
	     71,  70,  70,  69,  68,  68,  67,  66,  66,  65,  64,  64,  63,  63,  62,  61,  61,  60,  60,  59,
	     59,  56,  53,  50,  47,  43,  40,  38,  36,  33,  30,  27,  25,  22,  20,  18,  16,  13,  10,   8,
	      6,   3,   0
};


uint16_t eepromWriteP(const uint16_t address, const void* data, const uint16_t size) {
	/*
	 * The eeprom page size is 64 bytes.
	 * Physical page boundaries start at addresses that are integer
	 * multiples of the page size and end at addresses that are
	 * integer [multiples of page size] - 1;
	 */
	uint16_t n = 0;
	while (n < size) {
		I2c.start();
		I2c.sendAddress(SLA_W(0x50));
		I2c.sendByte(highByte(address + n));
		I2c.sendByte(lowByte(address + n));
		do {
			I2c.sendByte(pgm_read_byte(data + n));
			n++;
		} while ((n < size) && ((address + n) % 64));
		I2c.stop();
		delay(10);
	}
	return n;
}

uint16_t epromReadWord(uint16_t address) {
	union {
		uint8_t value8[];
		uint16_t value16;
	} value;
	I2c.start();
	I2c.sendAddress(SLA_W(0x50));
	I2c.sendByte(highByte(address));
	I2c.sendByte(lowByte(address));
	I2c.start();
	I2c.sendAddress(SLA_R(0x50));
	I2c.receiveByte(1, &value.value8[0]);
	I2c.receiveByte(0, &value.value8[1]);
	I2c.stop();
	return value.value16;
}

void fillEeprom() {
	uint16_t tempAddress = 0;
	lcd.clear();
	tempAddress = tempAddress + eepromWriteP(tempAddress, LiquidABV, sizeof(LiquidABV));
	lcd.print(tempAddress, HEX);
	VaporAddress = tempAddress;
	tempAddress = tempAddress + eepromWriteP(tempAddress, VaporABV, sizeof(VaporABV));
	lcd.setCursor(0,1);
	lcd.print(tempAddress, HEX);
}


void verifyEeprom() {
	uint16_t i;
	uint16_t errorCount = 0;
	lcd.clear();
	for (i = 0; i < sizeof(LiquidABV) / 2; ++i) {
		lcd.setCursor(0,0);
		lcd.print(i, DEC);
		if (pgm_read_word(&LiquidABV[i]) != epromReadWord( 0 + i * 2))
			errorCount++;
	lcd.setCursor(8,0);
	lcd.print(errorCount, DEC);
	}
	errorCount = 0;
	for (i = 0; i < sizeof(VaporABV) / 2; ++i) {
		lcd.setCursor(0,1);
		lcd.print(i, DEC);
		if (pgm_read_word(&VaporABV[i]) != epromReadWord(VaporAddress + i * 2))
			errorCount++;
	lcd.setCursor(8,1);
	lcd.print(errorCount, DEC);
	}

}


void setup() {
	I2c.begin();
	I2c.setSpeed(1);
	I2c.timeOut(10);
	lcd.begin();
	lcd.setColor(RgbLcdKeyShieldI2C::clWhite);

	// show splash
	lcd.printP(msgSplash1);
	lcd.setCursor(0,1);
	lcd.printP(msgSplash2);

	delay(3000);

	lcd.clear();
	lcd.printP(msgStart1);
	lcd.setCursor(0,1);
	lcd.printP(msgStart2);
	lcd.keySelect.onShortPress = fillEeprom;
	lcd.keyLeft.onShortPress = verifyEeprom;
	lcd.keyDown.onShortPress = decCounter;
	lcd.keyDown.onRepPress = decCounter;
	lcd.keyUp.onShortPress = incCounter;
	lcd.keyUp.onRepPress = incCounter;
}

// The loop function is called in an endless loop
void loop() {
	lcd.readKeys();
}


void printTableRow() {
	lcd.setCursor(0,0);
	sprintf(lineBuffer, "%.4d %.4d %.4d  ", Counter, pgm_read_word(&LiquidABV[Counter]) , epromReadWord(Counter * 2));
	lcd.print(lineBuffer);
	lcd.setCursor(0,1);
	sprintf(lineBuffer, "%.4d %.4d %.4d  ", Counter, pgm_read_word(&VaporABV[Counter]) , epromReadWord(VaporAddress + Counter * 2));
	lcd.print(lineBuffer);

}

void incCounter() {
	Counter++;
	printTableRow();
}

void decCounter() {
	Counter--;
	printTableRow();
}




