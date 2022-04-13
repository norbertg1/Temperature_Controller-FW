/*
 * NTC_lookup_table.h
 *
 *	Values must be in ohms!
 *  Created on: 2020. dec. 18.
 *      Author: Norbert
 */

#ifndef INC_NTC_LOOKUP_TABLE_H_
#define INC_NTC_LOOKUP_TABLE_H_

//NTCS0603E3222FMT
static const float NTCS0603E3222FMT_RT[206][2] = {
		{-55,  140977},
		{-54,  131709},
		{-53,  123113},
		{-52,  115136},
		{-51,  107729},
		{-50,  100848},
		{-49,  94453},
		{-48,  88506},
		{-47,  82973},
		{-46,  77823},
		{-45,  73027},
		{-44,  68559},
		{-43,  64394},
		{-42,  60510},
		{-41,  56885},
		{-40,  53502},
		{-39,  50343},
		{-38,  47391},
		{-37,  44631},
		{-36,  42051},
		{-35,  39637},
		{-34,  37377},
		{-33,  35261},
		{-32,  33279},
		{-31,  31421},
		{-30,  29679},
		{-29,  28045},
		{-28,  26512},
		{-27,  25073},
		{-26,  23721},
		{-25,  22451},
		{-24,  21257},
		{-23,  20134},
		{-22,  19078},
		{-21,  18084},
		{-20,  17148},
		{-19,  16267},
		{-18,  15437},
		{-17,  14654},
		{-16,  13916},
		{-15,  13220},
		{-14,  12563},
		{-13,  11943},
		{-12,  11358},
		{-11,  10805},
		{-10,  10283},
		{-9,  9789},
		{-8,  9321},
		{-7,  8880},
		{-6,  8461},
		{-5,  8066},
		{-4,  7691},
		{-3,  7336},
		{-2,  6999},
		{-1,  6680},
		{0,  6378},
		{1,  6091},
		{2,  5819},
		{3,  5561},
		{4,  5316},
		{5,  5083},
		{6,  4861},
		{7,  4651},
		{8,  4451},
		{9,  4261},
		{10,  4080},
		{11,  3908},
		{12,  3744},
		{13,  3589},
		{14,  3440},
		{15,  3299},
		{16,  3164},
		{17,  3036},
		{18,  2913},
		{19,  2796},
		{20,  2685},
		{21,  2579},
		{22,  2477},
		{23,  2380},
		{24,  2288},
		{25,  2200},
		{26,  2115},
		{27,  2034},
		{28,  1957},
		{29,  1884},
		{30,  1813},
		{31,  1745},
		{32,  1681},
		{33,  1619},
		{34,  1560},
		{35,  1503},
		{36,  1449},
		{37,  1397},
		{38,  1347},
		{39,  1299},
		{40,  1253},
		{41,  1209},
		{42,  1167},
		{43,  1127},
		{44,  1088},
		{45,  1051},
		{46,  1015},
		{47,  981},
		{48,  948},
		{49,  916},
		{50,  886},
		{51,  856},
		{52,  828},
		{53,  801},
		{54,  775},
		{55,  750},
		{56,  726},
		{57,  703},
		{58,  680},
		{59,  659},
		{60,  638},
		{61,  618},
		{62,  599},
		{63,  581},
		{64,  563},
		{65,  546},
		{66,  529},
		{67,  513},
		{68,  498},
		{69,  483},
		{70,  468},
		{71,  455},
		{72,  441},
		{73,  428},
		{74,  416},
		{75,  404},
		{76,  392},
		{77,  381},
		{78,  370},
		{79,  360},
		{80,  350},
		{81,  340},
		{82,  330},
		{83,  321},
		{84,  312},
		{85,  304},
		{86,  296},
		{87,  288},
		{88,  280},
		{89,  272},
		{90,  265},
		{91,  258},
		{92,  251},
		{93,  245},
		{94,  238},
		{95,  232},
		{96,  226},
		{97,  220},
		{98,  215},
		{99,  209},
		{100,  204},
		{101,  199},
		{102,  194},
		{103,  189},
		{104,  184},
		{105,  180},
		{106,  175},
		{107,  171},
		{108,  167},
		{109,  163},
		{110,  159},
		{111,  155},
		{112,  151},
		{113,  148},
		{114,  144},
		{115,  141},
		{116,  138},
		{117,  134},
		{118,  131},
		{119,  128},
		{120,  125},
		{121,  123},
		{122,  120},
		{123,  117},
		{124,  114},
		{125,  112},
		{126,  109},
		{127,  107},
		{128,  105},
		{129,  102},
		{130,  100},
		{131,  98},
		{132,  96},
		{133,  94},
		{134,  92},
		{135,  90},
		{136,  88},
		{137,  86},
		{138,  84},
		{139,  82},
		{140,  81},
		{141,  79},
		{142,  77},
		{143,  76},
		{144,  74},
		{145,  73},
		{146,  71},
		{147,  70},
		{148,  68},
		{149,  67},
		{150,  66}
};

static const float NTCG163JX103DTDS_RT[166][2] = {
		{-40,	188500},
		{-39,	178600},
		{-38,	169200},
		{-37,	160400},
		{-36,	152100},
		{-35,	144300},
		{-34,	136900},
		{-33,	130000},
		{-32,	123400},
		{-31,	117200},
		{-30,	111300},
		{-29,	105800},
		{-28,	100600},
		{-27,	95640},
		{-26,	90970},
		{-25,	86560},
		{-24,	82380},
		{-23,	78430},
		{-22,	74690},
		{-21,	71140},
		{-20,	67790},
		{-19,	64610},
		{-18,	61600},
		{-17,	58740},
		{-16,	56030},
		{-15,	53460},
		{-14,	51030},
		{-13,	48710},
		{-12,	46520},
		{-11,	44430},
		{-10,	42450},
		{-9,	40570},
		{-8,	38780},
		{-7,	37080},
		{-6,	35460},
		{-5,	33930},
		{-4,	32460},
		{-3,	31070},
		{-2,	29750},
		{-1,	28490},
		{0,	27280},
		{1,	26140},
		{2,	25050},
		{3,	24010},
		{4,	23020},
		{5,	22070},
		{6,	21170},
		{7,	20310},
		{8,	19490},
		{9,	18710},
		{10,	17960},
		{11,	17250},
		{12,	16570},
		{13,	15910},
		{14,	15290},
		{15,	14700},
		{16,	14130},
		{17,	13590},
		{18,	13070},
		{19,	12570},
		{20,	12090},
		{21,	11640},
		{22,	11200},
		{23,	10780},
		{24,	10380},
		{25,	10000},
		{26,	9633},
		{27,	9282},
		{28,	8945},
		{29,	8622},
		{30,	8312},
		{31,	8015},
		{32,	7730},
		{33,	7456},
		{34,	7194},
		{35,	6942},
		{36,	6700},
		{37,	6468},
		{38,	6245},
		{39,	6031},
		{40,	5826},
		{41,	5628},
		{42,	5438},
		{43,	5255},
		{44,	5080},
		{45,	4911},
		{46,	4749},
		{47,	4592},
		{48,	4442},
		{49,	4297},
		{50,	4158},
		{51,	4024},
		{52,	3895},
		{53,	3771},
		{54,	3651},
		{55,	3536},
		{56,	3425},
		{57,	3318},
		{58,	3215},
		{59,	3115},
		{60,	3019},
		{61,	2927},
		{62,	2837},
		{63,	2751},
		{64,	2668},
		{65,	2588},
		{66,	2511},
		{67,	2436},
		{68,	2364},
		{69,	2295},
		{70,	2227},
		{71,	2163},
		{72,	2100},
		{73,	2039},
		{74,	1981},
		{75,	1924},
		{76,	1869},
		{77,	1817},
		{78,	1765},
		{79,	1716},
		{80,	1668},
		{81,	1622},
		{82,	1577},
		{83,	1534},
		{84,	1492},
		{85,	1451},
		{86,	1412},
		{87,	1374},
		{88,	1337},
		{89,	1302},
		{90,	1267},
		{91,	1234},
		{92,	1201},
		{93,	1170},
		{94,	1139},
		{95,	1110},
		{96,	1081},
		{97,	1054},
		{98,	1027},
		{99,	1001},
		{100,	975},
		{101,	951},
		{102,	927},
		{103,	904},
		{104,	881},
		{105,	860},
		{106,	838},
		{107,	818},
		{108,	798},
		{109,	779},
		{110,	760},
		{111,	742},
		{112,	724},
		{113,	707},
		{114,	690},
		{115,	674},
		{116,	658},
		{117,	643},
		{118,	628},
		{119,	613},
		{120,	599},
		{121,	585},
		{122,	572},
		{123,	559},
		{124,	546},
		{125,	534}
};

static const float NTC_100K_RT[337][2] = {
		{-38,	2829200},
		{-37,	2654330},
		{-36,	2491360},
		{-35,	2339450},
		{-34,	2197760},
		{-33,	2065540},
		{-32,	1942100},
		{-31,	1826800},
		{-30,	1719040},
		{-29,	1618290},
		{-28,	1524050},
		{-27,	1435860},
		{-26,	1353290},
		{-25,	1275950},
		{-24,	1203480},
		{-23,	1135550},
		{-22,	1071830},
		{-21,	1012060},
		{-20,	955950},
		{-19,	903260},
		{-18,	853780},
		{-17,	807270},
		{-16,	763560},
		{-15,	722450},
		{-14,	683780},
		{-13,	647380},
		{-12,	613120},
		{-11,	580860},
		{-10,	550460},
		{-9,	521810},
		{-8,	494810},
		{-7,	469350},
		{-6,	445330},
		{-5,	422660},
		{-4,	401260},
		{-3,	381060},
		{-2,	361980},
		{-1,	343960},
		{0,	326920},
		{1,	310810},
		{2,	295580},
		{0,	326920},
		{1,	310810},
		{2,	295580},
		{3,	281170},
		{4,	267540},
		{5,	254640},
		{6,	242420},
		{7,	230860},
		{8,	219900},
		{9,	209520},
		{10,	199678},
		{11,	190349},
		{12,	181503},
		{13,	173111},
		{14,	165150},
		{15,	157594},
		{16,	150422},
		{17,	143611},
		{18,	137143},
		{19,	130998},
		{20,	125158},
		{21,	119608},
		{22,	114332},
		{23,	109314},
		{24,	104541},
		{25,	100000},
		{26,	95679},
		{27,	91565},
		{28,	87649},
		{29,	83920},
		{30,	80367},
		{31,	76982},
		{32,	73757},
		{33,	70682},
		{34,	67751},
		{35,	64956},
		{36,	62289},
		{37,	59745},
		{38,	57318},
		{39,	55001},
		{40,	52789},
		{41,	50677},
		{42,	48659},
		{43,	46732},
		{44,	44890},
		{45,	43130},
		{46,	41448},
		{47,	39839},
		{48,	38301},
		{49,	36829},
		{50,	35421},
		{51,	34074},
		{52,	32785},
		{53,	31550},
		{54,	30368},
		{55,	29236},
		{56,	28152},
		{57,	27113},
		{58,	26118},
		{59,	25163},
		{60,	24249},
		{61,	23372},
		{62,	22530},
		{63,	21723},
		{64,	20949},
		{65,	20207},
		{66,	19494},
		{67,	18809},
		{68,	18152},
		{69,	17521},
		{70,	16915},
		{71,	16333},
		{72,	15773},
		{73,	15235},
		{74,	14719},
		{75,	14222},
		{76,	13744.1},
		{77,	13284.6},
		{78,	12842.7},
		{79,	12417.5},
		{80,	12008.3},
		{81,	11614.6},
		{82,	11235.6},
		{83,	10870.7},
		{84,	10519.4},
		{85,	10181},
		{86,	9855.1},
		{87,	9541.1},
		{88,	9238.6},
		{89,	8947},
		{90,	8666},
		{91,	8395},
		{92,	8133.8},
		{93,	7881.9},
		{94,	7639},
		{95,	7404.6},
		{96,	7178.5},
		{97,	6960.3},
		{98,	6749.7},
		{99,	6546.4},
		{100,	6350.2},
		{101,	6160.7},
		{102,	5977.7},
		{103,	5801},
		{104,	5630.2},
		{105,	5465.3},
		{106,	5305.9},
		{107,	5151.9},
		{108,	5003},
		{109,	4859.1},
		{110,	4719.9},
		{111,	4585.3},
		{112,	4455.2},
		{113,	4329.3},
		{114,	4207.5},
		{115,	4089.7},
		{116,	3975.7},
		{117,	3865.4},
		{118,	3758.5},
		{119,	3655.2},
		{120,	3555},
		{121,	3458.1},
		{122,	3364.2},
		{123,	3273.3},
		{124,	3185.2},
		{125,	3099.9},
		{126,	3017.2},
		{127,	2937},
		{128,	2859.4},
		{129,	2784.1},
		{130,	2711.1},
		{131,	2640.3},
		{132,	2571.7},
		{133,	2505.1},
		{134,	2440.6},
		{135,	2377.9},
		{136,	2317.2},
		{137,	2258.2},
		{138,	2201},
		{141,	2039.3},
		{142,	1988.5},
		{143,	1939.2},
		{144,	1891.3},
		{145,	1844.8},
		{146,	1799.7},
		{147,	1755.8},
		{148,	1713.2},
		{149,	1671.7},
		{150,	1631.5},
		{151,	1592.4},
		{152,	1554.3},
		{153,	1517.4},
		{154,	1481.5},
		{155,	1446.5},
		{156,	1412.6},
		{157,	1379.5},
		{158,	1347.4},
		{159,	1316.2},
		{160,	1285.8},
		{161,	1256.2},
		{162,	1227.4},
		{163,	1199.4},
		{164,	1172.2},
		{165,	1145.7},
		{166,	1119.9},
		{167,	1094.7},
		{168,	1070.3},
		{169,	1046.5},
		{170,	1023.3},
		{171,	1000.7},
		{172,	978.7},
		{173,	957.3},
		{174,	936.4},
		{175,	916.1},
		{176,	896.3},
		{177,	876.9},
		{178,	858.1},
		{179,	839.8},
		{180,	821.9},
		{181,	804.5},
		{182,	787.5},
		{183,	771},
		{184,	754.8},
		{185,	739.1},
		{186,	723.7},
		{187,	708.8},
		{188,	694.2},
		{189,	679.9},
		{190,	666},
		{191,	652.5},
		{192,	639.2},
		{193,	626.3},
		{194,	613.7},
		{195,	601.4},
		{196,	589.4},
		{197,	577.7},
		{198,	566.3},
		{201,	533.5},
		{202,	523.1},
		{203,	513},
		{204,	503.1},
		{205,	493.4},
		{206,	483.9},
		{207,	474.6},
		{208,	465.6},
		{209,	456.8},
		{210,	448.1},
		{211,	439.7},
		{212,	431.4},
		{213,	423.4},
		{214,	415.5},
		{215,	407.8},
		{216,	400.3},
		{217,	392.9},
		{218,	385.7},
		{219,	378.6},
		{220,	371.7},
		{221,	365},
		{222,	358.4},
		{223,	352},
		{224,	345.6},
		{225,	339.5},
		{226,	333.4},
		{227,	327.5},
		{228,	321.7},
		{229,	316.1},
		{230,	310.5},
		{231,	305.1},
		{232,	299.8},
		{233,	294.6},
		{234,	289.5},
		{235,	284.5},
		{236,	279.6},
		{237,	274.8},
		{238,	270.1},
		{239,	265.5},
		{240,	261},
		{241,	256.6},
		{242,	252.3},
		{243,	248.1},
		{244,	243.9},
		{245,	239.9},
		{246,	235.9},
		{247,	232},
		{248,	228.2},
		{249,	224.4},
		{250,	220.7},
		{251,	217.1},
		{252,	213.6},
		{253,	210.1},
		{254,	206.7},
		{255,	203.4},
		{256,	200.2},
		{257,	197},
		{258,	193.8},
		{259,	190.7},
		{261,	184.8},
		{262,	181.9},
		{263,	179},
		{264,	176.2},
		{265,	173.5},
		{266,	170.8},
		{267,	168.2},
		{268,	165.6},
		{269,	163},
		{270,	160.5},
		{271,	158.1},
		{272,	155.7},
		{273,	153.3},
		{274,	151},
		{275,	148.7},
		{276,	146.5},
		{277,	144.3},
		{278,	142.2},
		{279,	140.1},
		{280,	138},
		{281,	136},
		{282,	134},
		{283,	132},
		{284,	130.1},
		{285,	128.2},
		{286,	126.4},
		{287,	124.6},
		{288,	122.8},
		{289,	121},
		{290,	119.3},
		{291,	117.6},
		{292,	116},
		{293,	114.4},
		{294,	112.8},
		{295,	111.2},
		{296,	109.7},
		{297,	108.2},
		{298,	106.7},
		{299,	105.2},
		{300,	103.8}
};

static const float PT100_RT[651][2] = {
		{-150,	42.68515},
		{-149,	43.05860402},
		{-148,	43.43217408},
		{-147,	43.80586018},
		{-146,	44.17966232},
		{-145,	44.5535805},
		{-144,	44.92761472},
		{-143,	45.30176498},
		{-142,	45.67603128},
		{-141,	46.05041362},
		{-140,	46.424912},
		{-139,	46.79952642},
		{-138,	47.17425688},
		{-137,	47.54910338},
		{-136,	47.92406592},
		{-135,	48.2991445},
		{-134,	48.67433912},
		{-133,	49.04964978},
		{-132,	49.42507648},
		{-131,	49.80061922},
		{-130,	50.176278},
		{-129,	50.55205282},
		{-128,	50.92794368},
		{-127,	51.30395058},
		{-126,	51.68007352},
		{-125,	52.0563125},
		{-124,	52.43266752},
		{-123,	52.80913858},
		{-122,	53.18572568},
		{-121,	53.56242882},
		{-120,	53.939248},
		{-119,	54.31618322},
		{-118,	54.69323448},
		{-117,	55.07040178},
		{-116,	55.44768512},
		{-115,	55.8250845},
		{-114,	56.20259992},
		{-113,	56.58023138},
		{-112,	56.95797888},
		{-111,	57.33584242},
		{-110,	57.713822},
		{-109,	58.09191762},
		{-108,	58.47012928},
		{-107,	58.84845698},
		{-106,	59.22690072},
		{-105,	59.6054605},
		{-104,	59.98413632},
		{-103,	60.36292818},
		{-102,	60.74183608},
		{-101,	61.12086002},
		{-100,	61.5},
		{-99,	61.87925602},
		{-98,	62.25862808},
		{-97,	62.63811618},
		{-96,	63.01772032},
		{-95,	63.3974405},
		{-94,	63.77727672},
		{-93,	64.15722898},
		{-92,	64.53729728},
		{-91,	64.91748162},
		{-90,	65.297782},
		{-89,	65.67819842},
		{-88,	66.05873088},
		{-87,	66.43937938},
		{-86,	66.82014392},
		{-85,	67.2010245},
		{-84,	67.58202112},
		{-83,	67.96313378},
		{-82,	68.34436248},
		{-81,	68.72570722},
		{-80,	69.107168},
		{-79,	69.48874482},
		{-78,	69.87043768},
		{-77,	70.25224658},
		{-76,	70.63417152},
		{-75,	71.0162125},
		{-74,	71.39836952},
		{-73,	71.78064258},
		{-72,	72.16303168},
		{-71,	72.54553682},
		{-70,	72.928158},
		{-69,	73.31089522},
		{-68,	73.69374848},
		{-67,	74.07671778},
		{-66,	74.45980312},
		{-65,	74.8430045},
		{-64,	75.22632192},
		{-63,	75.60975538},
		{-62,	75.99330488},
		{-61,	76.37697042},
		{-60,	76.760752},
		{-59,	77.14464962},
		{-58,	77.52866328},
		{-57,	77.91279298},
		{-56,	78.29703872},
		{-55,	78.6814005},
		{-54,	79.06587832},
		{-53,	79.45047218},
		{-52,	79.83518208},
		{-51,	80.22000802},
		{-50,	80.31},
		{-49,	80.7},
		{-48,	81.1},
		{-47,	81.5},
		{-46,	81.89},
		{-45,	82.29},
		{-44,	82.69},
		{-43,	83.08},
		{-42,	83.48},
		{-41,	83.88},
		{-40,	84.27},
		{-39,	84.67},
		{-38,	85.06},
		{-37,	85.46},
		{-36,	85.85},
		{-35,	86.25},
		{-34,	86.64},
		{-33,	87.04},
		{-32,	87.43},
		{-31,	87.83},
		{-30,	88.22},
		{-29,	88.62},
		{-28,	89.01},
		{-27,	89.4},
		{-26,	89.8},
		{-25,	90.19},
		{-24,	90.59},
		{-23,	90.98},
		{-22,	91.37},
		{-21,	91.77},
		{-20,	92.16},
		{-19,	92.55},
		{-18,	92.95},
		{-17,	93.34},
		{-16,	93.73},
		{-15,	94.12},
		{-14,	94.52},
		{-13,	94.91},
		{-12,	95.3},
		{-11,	95.69},
		{-10,	96.09},
		{-9,	96.48},
		{-8,	96.87},
		{-7,	97.26},
		{-6,	97.65},
		{-5,	98.04},
		{-4,	98.44},
		{-3,	98.83},
		{-2,	99.22},
		{-1,	99.61},
		{0,	100},
		{1,	100.39},
		{2,	100.78},
		{3,	101.17},
		{4,	101.56},
		{5,	101.95},
		{6,	102.34},
		{7,	102.73},
		{8,	103.12},
		{9,	103.51},
		{10,	103.9},
		{11,	104.29},
		{12,	104.68},
		{13,	105.07},
		{14,	105.46},
		{15,	105.85},
		{16,	106.24},
		{17,	106.63},
		{18,	107.02},
		{19,	107.4},
		{20,	107.79},
		{21,	108.18},
		{22,	108.57},
		{23,	108.96},
		{24,	109.35},
		{25,	109.73},
		{26,	110.12},
		{27,	110.51},
		{28,	110.9},
		{29,	111.28},
		{30,	111.67},
		{31,	112.06},
		{32,	112.45},
		{33,	112.83},
		{34,	113.22},
		{35,	113.61},
		{36,	113.99},
		{37,	114.38},
		{38,	114.77},
		{39,	115.15},
		{40,	115.54},
		{41,	115.93},
		{42,	116.31},
		{43,	116.7},
		{44,	117.08},
		{45,	117.47},
		{46,	117.85},
		{47,	118.24},
		{48,	118.62},
		{49,	119.01},
		{50,	119.4},
		{51,	119.78},
		{52,	120.16},
		{53,	120.55},
		{54,	120.93},
		{55,	121.32},
		{56,	121.7},
		{57,	122.09},
		{58,	122.47},
		{59,	122.86},
		{60,	123.24},
		{61,	123.62},
		{62,	124.01},
		{63,	124.39},
		{64,	124.77},
		{65,	125.16},
		{66,	125.54},
		{67,	125.92},
		{68,	126.31},
		{69,	126.69},
		{70,	127.07},
		{71,	127.45},
		{72,	127.84},
		{73,	128.22},
		{74,	128.6},
		{75,	128.98},
		{76,	129.37},
		{77,	129.75},
		{78,	130.13},
		{79,	130.51},
		{80,	130.89},
		{81,	131.27},
		{82,	131.66},
		{83,	132.04},
		{84,	132.42},
		{85,	132.8},
		{86,	133.18},
		{87,	133.56},
		{88,	133.94},
		{89,	134.32},
		{90,	134.7},
		{91,	135.08},
		{92,	135.46},
		{93,	135.84},
		{94,	136.22},
		{95,	136.6},
		{96,	136.98},
		{97,	137.36},
		{98,	137.74},
		{99,	138.12},
		{100,	138.5},
		{101,	138.88},
		{102,	139.26},
		{103,	139.64},
		{104,	140.02},
		{105,	140.39},
		{106,	140.77},
		{107,	141.15},
		{108,	141.53},
		{109,	141.91},
		{110,	142.29},
		{111,	142.66},
		{112,	143.04},
		{113,	143.42},
		{114,	143.8},
		{115,	144.17},
		{116,	144.55},
		{117,	144.93},
		{118,	145.31},
		{119,	145.68},
		{120,	146.06},
		{121,	146.44},
		{122,	146.81},
		{123,	147.19},
		{124,	147.57},
		{125,	147.94},
		{126,	148.32},
		{127,	148.7},
		{128,	149.07},
		{129,	149.45},
		{130,	149.82},
		{131,	150.2},
		{132,	150.57},
		{133,	150.95},
		{134,	151.33},
		{135,	151.7},
		{136,	152.08},
		{137,	152.45},
		{138,	152.83},
		{139,	153.2},
		{140,	153.58},
		{141,	153.95},
		{142,	154.32},
		{143,	154.7},
		{144,	155.07},
		{145,	155.45},
		{146,	155.82},
		{147,	156.19},
		{148,	156.57},
		{149,	156.94},
		{150,	157.31},
		{151,	157.69},
		{152,	158.06},
		{153,	158.43},
		{154,	158.81},
		{155,	159.18},
		{156,	159.55},
		{157,	159.93},
		{158,	160.3},
		{159,	160.67},
		{160,	161.04},
		{161,	161.42},
		{162,	161.79},
		{163,	162.16},
		{164,	162.53},
		{165,	162.9},
		{166,	163.27},
		{167,	163.65},
		{168,	164.02},
		{169,	164.39},
		{170,	164.76},
		{171,	165.13},
		{172,	165.5},
		{173,	165.87},
		{174,	166.24},
		{175,	166.61},
		{176,	166.98},
		{177,	167.35},
		{178,	167.72},
		{179,	168.09},
		{180,	168.46},
		{181,	168.83},
		{182,	169.2},
		{183,	169.57},
		{184,	169.94},
		{185,	170.31},
		{186,	170.68},
		{187,	171.05},
		{188,	171.42},
		{189,	171.79},
		{190,	172.16},
		{191,	172.53},
		{192,	172.9},
		{193,	173.26},
		{194,	173.63},
		{195,	174},
		{196,	174.37},
		{197,	174.74},
		{198,	175.1},
		{199,	175.47},
		{200,	175.84},
		{201,	176.21},
		{202,	176.57},
		{203,	176.94},
		{204,	177.31},
		{205,	177.68},
		{206,	178.04},
		{207,	178.41},
		{208,	178.78},
		{209,	179.14},
		{210,	179.51},
		{211,	179.88},
		{212,	180.24},
		{213,	180.61},
		{214,	180.97},
		{215,	181.34},
		{216,	181.71},
		{217,	182.07},
		{218,	182.44},
		{219,	182.8},
		{220,	183.17},
		{221,	183.53},
		{222,	183.9},
		{223,	184.26},
		{224,	184.63},
		{225,	184.99},
		{226,	185.36},
		{227,	185.72},
		{228,	186.09},
		{229,	186.45},
		{230,	186.82},
		{231,	187.18},
		{232,	187.54},
		{233,	187.91},
		{234,	188.27},
		{235,	188.63},
		{236,	189},
		{237,	189.36},
		{238,	189.72},
		{239,	190.09},
		{240,	190.45},
		{241,	190.81},
		{242,	191.18},
		{243,	191.54},
		{244,	191.9},
		{245,	192.26},
		{246,	192.63},
		{247,	192.99},
		{248,	193.35},
		{249,	193.71},
		{250,	194.07},
		{251,	194.44},
		{252,	194.8},
		{253,	195.16},
		{254,	195.52},
		{255,	195.88},
		{256,	196.24},
		{257,	196.6},
		{258,	196.96},
		{259,	197.33},
		{260,	197.69},
		{261,	198.05},
		{262,	198.41},
		{263,	198.77},
		{264,	199.13},
		{265,	199.49},
		{266,	199.85},
		{267,	200.21},
		{268,	200.57},
		{269,	200.93},
		{270,	201.29},
		{271,	201.65},
		{272,	202.01},
		{273,	202.36},
		{274,	202.72},
		{275,	203.08},
		{276,	203.44},
		{277,	203.8},
		{278,	204.16},
		{279,	204.52},
		{280,	204.88},
		{281,	205.23},
		{282,	205.59},
		{283,	205.95},
		{284,	206.31},
		{285,	206.67},
		{286,	207.02},
		{287,	207.38},
		{288,	207.74},
		{289,	208.1},
		{290,	208.45},
		{291,	208.81},
		{292,	209.17},
		{293,	209.52},
		{294,	209.88},
		{295,	210.24},
		{296,	210.59},
		{297,	210.95},
		{298,	211.31},
		{299,	211.66},
		{300,	212.02},
		{301,	212.37},
		{302,	212.73},
		{303,	213.09},
		{304,	213.44},
		{305,	213.8},
		{306,	214.15},
		{307,	214.51},
		{308,	214.86},
		{309,	215.22},
		{310,	215.57},
		{311,	215.93},
		{312,	216.28},
		{313,	216.64},
		{314,	216.99},
		{315,	217.35},
		{316,	217.7},
		{317,	218.05},
		{318,	218.41},
		{319,	218.76},
		{320,	219.12},
		{321,	219.47},
		{322,	219.82},
		{323,	220.18},
		{324,	220.53},
		{325,	220.88},
		{326,	221.24},
		{327,	221.59},
		{328,	221.94},
		{329,	222.29},
		{330,	222.65},
		{331,	223},
		{332,	223.35},
		{333,	223.7},
		{334,	224.06},
		{335,	224.41},
		{336,	224.76},
		{337,	225.11},
		{338,	225.46},
		{339,	225.81},
		{340,	226.17},
		{341,	226.52},
		{342,	226.87},
		{343,	227.22},
		{344,	227.57},
		{345,	227.92},
		{346,	228.27},
		{347,	228.62},
		{348,	228.97},
		{349,	229.32},
		{350,	229.67},
		{351,	230.02},
		{352,	230.37},
		{353,	230.72},
		{354,	231.07},
		{355,	231.42},
		{356,	231.77},
		{357,	232.12},
		{358,	232.47},
		{359,	232.82},
		{360,	233.17},
		{361,	233.52},
		{362,	233.87},
		{363,	234.22},
		{364,	234.56},
		{365,	234.91},
		{366,	235.26},
		{367,	235.61},
		{368,	235.96},
		{369,	236.31},
		{370,	236.65},
		{371,	237},
		{372,	237.35},
		{373,	237.7},
		{374,	238.04},
		{375,	238.39},
		{376,	238.74},
		{377,	239.09},
		{378,	239.43},
		{379,	239.78},
		{380,	240.13},
		{381,	240.47},
		{382,	240.82},
		{383,	241.17},
		{384,	241.51},
		{385,	241.86},
		{386,	242.2},
		{387,	242.55},
		{388,	242.9},
		{389,	243.24},
		{390,	243.59},
		{391,	243.93},
		{392,	244.28},
		{393,	244.62},
		{394,	244.97},
		{395,	245.31},
		{396,	245.66},
		{397,	246},
		{398,	246.35},
		{399,	246.69},
		{400,	247.04},
		{401,	247.38},
		{402,	247.73},
		{403,	248.07},
		{404,	248.41},
		{405,	248.76},
		{406,	249.1},
		{407,	249.45},
		{408,	249.79},
		{409,	250.13},
		{410,	250.48},
		{411,	250.82},
		{412,	251.16},
		{413,	251.5},
		{414,	251.85},
		{415,	252.19},
		{416,	252.53},
		{417,	252.88},
		{418,	253.22},
		{419,	253.56},
		{420,	253.9},
		{421,	254.24},
		{422,	254.59},
		{423,	254.93},
		{424,	255.27},
		{425,	255.61},
		{426,	255.95},
		{427,	256.29},
		{428,	256.64},
		{429,	256.98},
		{430,	257.32},
		{431,	257.66},
		{432,	258},
		{433,	258.34},
		{434,	258.68},
		{435,	259.02},
		{436,	259.36},
		{437,	259.7},
		{438,	260.04},
		{439,	260.38},
		{440,	260.72},
		{441,	261.06},
		{442,	261.4},
		{443,	261.74},
		{444,	262.08},
		{445,	262.42},
		{446,	262.76},
		{447,	263.1},
		{448,	263.43},
		{449,	263.77},
		{450,	264.11},
		{451,	264.45},
		{452,	264.79},
		{453,	265.13},
		{454,	265.47},
		{455,	265.8},
		{456,	266.14},
		{457,	266.48},
		{458,	266.82},
		{459,	267.15},
		{460,	267.49},
		{461,	267.83},
		{462,	268.17},
		{463,	268.5},
		{464,	268.84},
		{465,	269.18},
		{466,	269.51},
		{467,	269.85},
		{468,	270.19},
		{469,	270.52},
		{470,	270.86},
		{471,	271.2},
		{472,	271.53},
		{473,	271.87},
		{474,	272.2},
		{475,	272.54},
		{476,	272.88},
		{477,	273.21},
		{478,	273.55},
		{479,	273.88},
		{480,	274.22},
		{481,	274.55},
		{482,	274.89},
		{483,	275.22},
		{484,	275.56},
		{485,	275.89},
		{486,	276.23},
		{487,	276.56},
		{488,	276.89},
		{489,	277.23},
		{490,	277.56},
		{491,	277.9},
		{492,	278.23},
		{493,	278.56},
		{494,	278.9},
		{495,	279.23},
		{496,	279.56},
		{497,	279.9},
		{498,	280.23},
		{499,	280.56},
		{500,	280.9}
};

static const float PT1000_RT[651][2] = {
		{-150,	426.8515},
		{-149,	430.5860402},
		{-148,	434.3217408},
		{-147,	438.0586018},
		{-146,	441.7966232},
		{-145,	445.535805},
		{-144,	449.2761472},
		{-143,	453.0176498},
		{-142,	456.7603128},
		{-141,	460.5041362},
		{-140,	464.24912},
		{-139,	467.9952642},
		{-138,	471.7425688},
		{-137,	475.4910338},
		{-136,	479.2406592},
		{-135,	482.991445},
		{-134,	486.7433912},
		{-133,	490.4964978},
		{-132,	494.2507648},
		{-131,	498.0061922},
		{-130,	501.76278},
		{-129,	505.5205282},
		{-128,	509.2794368},
		{-127,	513.0395058},
		{-126,	516.8007352},
		{-125,	520.563125},
		{-124,	524.3266752},
		{-123,	528.0913858},
		{-122,	531.8572568},
		{-121,	535.6242882},
		{-120,	539.39248},
		{-119,	543.1618322},
		{-118,	546.9323448},
		{-117,	550.7040178},
		{-116,	554.4768512},
		{-115,	558.250845},
		{-114,	562.0259992},
		{-113,	565.8023138},
		{-112,	569.5797888},
		{-111,	573.3584242},
		{-110,	577.13822},
		{-109,	580.9191762},
		{-108,	584.7012928},
		{-107,	588.4845698},
		{-106,	592.2690072},
		{-105,	596.054605},
		{-104,	599.8413632},
		{-103,	603.6292818},
		{-102,	607.4183608},
		{-101,	611.2086002},
		{-100,	615},
		{-99,	618.7925602},
		{-98,	622.5862808},
		{-97,	626.3811618},
		{-96,	630.1772032},
		{-95,	633.974405},
		{-94,	637.7727672},
		{-93,	641.5722898},
		{-92,	645.3729728},
		{-91,	649.1748162},
		{-90,	652.97782},
		{-89,	656.7819842},
		{-88,	660.5873088},
		{-87,	664.3937938},
		{-86,	668.2014392},
		{-85,	672.010245},
		{-84,	675.8202112},
		{-83,	679.6313378},
		{-82,	683.4436248},
		{-81,	687.2570722},
		{-80,	691.07168},
		{-79,	694.8874482},
		{-78,	698.7043768},
		{-77,	702.5224658},
		{-76,	706.3417152},
		{-75,	710.162125},
		{-74,	713.9836952},
		{-73,	717.8064258},
		{-72,	721.6303168},
		{-71,	725.4553682},
		{-70,	729.28158},
		{-69,	733.1089522},
		{-68,	736.9374848},
		{-67,	740.7671778},
		{-66,	744.5980312},
		{-65,	748.430045},
		{-64,	752.2632192},
		{-63,	756.0975538},
		{-62,	759.9330488},
		{-61,	763.7697042},
		{-60,	767.60752},
		{-59,	771.4464962},
		{-58,	775.2866328},
		{-57,	779.1279298},
		{-56,	782.9703872},
		{-55,	786.814005},
		{-54,	790.6587832},
		{-53,	794.5047218},
		{-52,	798.3518208},
		{-51,	802.2000802},
		{-50,	803.1},
		{-49,	807},
		{-48,	811},
		{-47,	815},
		{-46,	818.9},
		{-45,	822.9},
		{-44,	826.9},
		{-43,	830.8},
		{-42,	834.8},
		{-41,	838.8},
		{-40,	842.7},
		{-39,	846.7},
		{-38,	850.6},
		{-37,	854.6},
		{-36,	858.5},
		{-35,	862.5},
		{-34,	866.4},
		{-33,	870.4},
		{-32,	874.3},
		{-31,	878.3},
		{-30,	882.2},
		{-29,	886.2},
		{-28,	890.1},
		{-27,	894},
		{-26,	898},
		{-25,	901.9},
		{-24,	905.9},
		{-23,	909.8},
		{-22,	913.7},
		{-21,	917.7},
		{-20,	921.6},
		{-19,	925.5},
		{-18,	929.5},
		{-17,	933.4},
		{-16,	937.3},
		{-15,	941.2},
		{-14,	945.2},
		{-13,	949.1},
		{-12,	953},
		{-11,	956.9},
		{-10,	960.9},
		{-9,	964.8},
		{-8,	968.7},
		{-7,	972.6},
		{-6,	976.5},
		{-5,	980.4},
		{-4,	984.4},
		{-3,	988.3},
		{-2,	992.2},
		{-1,	996.1},
		{0,	1000},
		{1,	1003.9},
		{2,	1007.8},
		{3,	1011.7},
		{4,	1015.6},
		{5,	1019.5},
		{6,	1023.4},
		{7,	1027.3},
		{8,	1031.2},
		{9,	1035.1},
		{10,	1039},
		{11,	1042.9},
		{12,	1046.8},
		{13,	1050.7},
		{14,	1054.6},
		{15,	1058.5},
		{16,	1062.4},
		{17,	1066.3},
		{18,	1070.2},
		{19,	1074},
		{20,	1077.9},
		{21,	1081.8},
		{22,	1085.7},
		{23,	1089.6},
		{24,	1093.5},
		{25,	1097.3},
		{26,	1101.2},
		{27,	1105.1},
		{28,	1109},
		{29,	1112.8},
		{30,	1116.7},
		{31,	1120.6},
		{32,	1124.5},
		{33,	1128.3},
		{34,	1132.2},
		{35,	1136.1},
		{36,	1139.9},
		{37,	1143.8},
		{38,	1147.7},
		{39,	1151.5},
		{40,	1155.4},
		{41,	1159.3},
		{42,	1163.1},
		{43,	1167},
		{44,	1170.8},
		{45,	1174.7},
		{46,	1178.5},
		{47,	1182.4},
		{48,	1186.2},
		{49,	1190.1},
		{50,	1194},
		{51,	1197.8},
		{52,	1201.6},
		{53,	1205.5},
		{54,	1209.3},
		{55,	1213.2},
		{56,	1217},
		{57,	1220.9},
		{58,	1224.7},
		{59,	1228.6},
		{60,	1232.4},
		{61,	1236.2},
		{62,	1240.1},
		{63,	1243.9},
		{64,	1247.7},
		{65,	1251.6},
		{66,	1255.4},
		{67,	1259.2},
		{68,	1263.1},
		{69,	1266.9},
		{70,	1270.7},
		{71,	1274.5},
		{72,	1278.4},
		{73,	1282.2},
		{74,	1286},
		{75,	1289.8},
		{76,	1293.7},
		{77,	1297.5},
		{78,	1301.3},
		{79,	1305.1},
		{80,	1308.9},
		{81,	1312.7},
		{82,	1316.6},
		{83,	1320.4},
		{84,	1324.2},
		{85,	1328},
		{86,	1331.8},
		{87,	1335.6},
		{88,	1339.4},
		{89,	1343.2},
		{90,	1347},
		{91,	1350.8},
		{92,	1354.6},
		{93,	1358.4},
		{94,	1362.2},
		{95,	1366},
		{96,	1369.8},
		{97,	1373.6},
		{98,	1377.4},
		{99,	1381.2},
		{100,	1385},
		{101,	1388.8},
		{102,	1392.6},
		{103,	1396.4},
		{104,	1400.2},
		{105,	1403.9},
		{106,	1407.7},
		{107,	1411.5},
		{108,	1415.3},
		{109,	1419.1},
		{110,	1422.9},
		{111,	1426.6},
		{112,	1430.4},
		{113,	1434.2},
		{114,	1438},
		{115,	1441.7},
		{116,	1445.5},
		{117,	1449.3},
		{118,	1453.1},
		{119,	1456.8},
		{120,	1460.6},
		{121,	1464.4},
		{122,	1468.1},
		{123,	1471.9},
		{124,	1475.7},
		{125,	1479.4},
		{126,	1483.2},
		{127,	1487},
		{128,	1490.7},
		{129,	1494.5},
		{130,	1498.2},
		{131,	1502},
		{132,	1505.7},
		{133,	1509.5},
		{134,	1513.3},
		{135,	1517},
		{136,	1520.8},
		{137,	1524.5},
		{138,	1528.3},
		{139,	1532},
		{140,	1535.8},
		{141,	1539.5},
		{142,	1543.2},
		{143,	1547},
		{144,	1550.7},
		{145,	1554.5},
		{146,	1558.2},
		{147,	1561.9},
		{148,	1565.7},
		{149,	1569.4},
		{150,	1573.1},
		{151,	1576.9},
		{152,	1580.6},
		{153,	1584.3},
		{154,	1588.1},
		{155,	1591.8},
		{156,	1595.5},
		{157,	1599.3},
		{158,	1603},
		{159,	1606.7},
		{160,	1610.4},
		{161,	1614.2},
		{162,	1617.9},
		{163,	1621.6},
		{164,	1625.3},
		{165,	1629},
		{166,	1632.7},
		{167,	1636.5},
		{168,	1640.2},
		{169,	1643.9},
		{170,	1647.6},
		{171,	1651.3},
		{172,	1655},
		{173,	1658.7},
		{174,	1662.4},
		{175,	1666.1},
		{176,	1669.8},
		{177,	1673.5},
		{178,	1677.2},
		{179,	1680.9},
		{180,	1684.6},
		{181,	1688.3},
		{182,	1692},
		{183,	1695.7},
		{184,	1699.4},
		{185,	1703.1},
		{186,	1706.8},
		{187,	1710.5},
		{188,	1714.2},
		{189,	1717.9},
		{190,	1721.6},
		{191,	1725.3},
		{192,	1729},
		{193,	1732.6},
		{194,	1736.3},
		{195,	1740},
		{196,	1743.7},
		{197,	1747.4},
		{198,	1751},
		{199,	1754.7},
		{200,	1758.4},
		{201,	1762.1},
		{202,	1765.7},
		{203,	1769.4},
		{204,	1773.1},
		{205,	1776.8},
		{206,	1780.4},
		{207,	1784.1},
		{208,	1787.8},
		{209,	1791.4},
		{210,	1795.1},
		{211,	1798.8},
		{212,	1802.4},
		{213,	1806.1},
		{214,	1809.7},
		{215,	1813.4},
		{216,	1817.1},
		{217,	1820.7},
		{218,	1824.4},
		{219,	1828},
		{220,	1831.7},
		{221,	1835.3},
		{222,	1839},
		{223,	1842.6},
		{224,	1846.3},
		{225,	1849.9},
		{226,	1853.6},
		{227,	1857.2},
		{228,	1860.9},
		{229,	1864.5},
		{230,	1868.2},
		{231,	1871.8},
		{232,	1875.4},
		{233,	1879.1},
		{234,	1882.7},
		{235,	1886.3},
		{236,	1890},
		{237,	1893.6},
		{238,	1897.2},
		{239,	1900.9},
		{240,	1904.5},
		{241,	1908.1},
		{242,	1911.8},
		{243,	1915.4},
		{244,	1919},
		{245,	1922.6},
		{246,	1926.3},
		{247,	1929.9},
		{248,	1933.5},
		{249,	1937.1},
		{250,	1940.7},
		{251,	1944.4},
		{252,	1948},
		{253,	1951.6},
		{254,	1955.2},
		{255,	1958.8},
		{256,	1962.4},
		{257,	1966},
		{258,	1969.6},
		{259,	1973.3},
		{260,	1976.9},
		{261,	1980.5},
		{262,	1984.1},
		{263,	1987.7},
		{264,	1991.3},
		{265,	1994.9},
		{266,	1998.5},
		{267,	2002.1},
		{268,	2005.7},
		{269,	2009.3},
		{270,	2012.9},
		{271,	2016.5},
		{272,	2020.1},
		{273,	2023.6},
		{274,	2027.2},
		{275,	2030.8},
		{276,	2034.4},
		{277,	2038},
		{278,	2041.6},
		{279,	2045.2},
		{280,	2048.8},
		{281,	2052.3},
		{282,	2055.9},
		{283,	2059.5},
		{284,	2063.1},
		{285,	2066.7},
		{286,	2070.2},
		{287,	2073.8},
		{288,	2077.4},
		{289,	2081},
		{290,	2084.5},
		{291,	2088.1},
		{292,	2091.7},
		{293,	2095.2},
		{294,	2098.8},
		{295,	2102.4},
		{296,	2105.9},
		{297,	2109.5},
		{298,	2113.1},
		{299,	2116.6},
		{300,	2120.2},
		{301,	2123.7},
		{302,	2127.3},
		{303,	2130.9},
		{304,	2134.4},
		{305,	2138},
		{306,	2141.5},
		{307,	2145.1},
		{308,	2148.6},
		{309,	2152.2},
		{310,	2155.7},
		{311,	2159.3},
		{312,	2162.8},
		{313,	2166.4},
		{314,	2169.9},
		{315,	2173.5},
		{316,	2177},
		{317,	2180.5},
		{318,	2184.1},
		{319,	2187.6},
		{320,	2191.2},
		{321,	2194.7},
		{322,	2198.2},
		{323,	2201.8},
		{324,	2205.3},
		{325,	2208.8},
		{326,	2212.4},
		{327,	2215.9},
		{328,	2219.4},
		{329,	2222.9},
		{330,	2226.5},
		{331,	2230},
		{332,	2233.5},
		{333,	2237},
		{334,	2240.6},
		{335,	2244.1},
		{336,	2247.6},
		{337,	2251.1},
		{338,	2254.6},
		{339,	2258.1},
		{340,	2261.7},
		{341,	2265.2},
		{342,	2268.7},
		{343,	2272.2},
		{344,	2275.7},
		{345,	2279.2},
		{346,	2282.7},
		{347,	2286.2},
		{348,	2289.7},
		{349,	2293.2},
		{350,	2296.7},
		{351,	2300.2},
		{352,	2303.7},
		{353,	2307.2},
		{354,	2310.7},
		{355,	2314.2},
		{356,	2317.7},
		{357,	2321.2},
		{358,	2324.7},
		{359,	2328.2},
		{360,	2331.7},
		{361,	2335.2},
		{362,	2338.7},
		{363,	2342.2},
		{364,	2345.6},
		{365,	2349.1},
		{366,	2352.6},
		{367,	2356.1},
		{368,	2359.6},
		{369,	2363.1},
		{370,	2366.5},
		{371,	2370},
		{372,	2373.5},
		{373,	2377},
		{374,	2380.4},
		{375,	2383.9},
		{376,	2387.4},
		{377,	2390.9},
		{378,	2394.3},
		{379,	2397.8},
		{380,	2401.3},
		{381,	2404.7},
		{382,	2408.2},
		{383,	2411.7},
		{384,	2415.1},
		{385,	2418.6},
		{386,	2422},
		{387,	2425.5},
		{388,	2429},
		{389,	2432.4},
		{390,	2435.9},
		{391,	2439.3},
		{392,	2442.8},
		{393,	2446.2},
		{394,	2449.7},
		{395,	2453.1},
		{396,	2456.6},
		{397,	2460},
		{398,	2463.5},
		{399,	2466.9},
		{400,	2470.4},
		{401,	2473.8},
		{402,	2477.3},
		{403,	2480.7},
		{404,	2484.1},
		{405,	2487.6},
		{406,	2491},
		{407,	2494.5},
		{408,	2497.9},
		{409,	2501.3},
		{410,	2504.8},
		{411,	2508.2},
		{412,	2511.6},
		{413,	2515},
		{414,	2518.5},
		{415,	2521.9},
		{416,	2525.3},
		{417,	2528.8},
		{418,	2532.2},
		{419,	2535.6},
		{420,	2539},
		{421,	2542.4},
		{422,	2545.9},
		{423,	2549.3},
		{424,	2552.7},
		{425,	2556.1},
		{426,	2559.5},
		{427,	2562.9},
		{428,	2566.4},
		{429,	2569.8},
		{430,	2573.2},
		{431,	2576.6},
		{432,	2580},
		{433,	2583.4},
		{434,	2586.8},
		{435,	2590.2},
		{436,	2593.6},
		{437,	2597},
		{438,	2600.4},
		{439,	2603.8},
		{440,	2607.2},
		{441,	2610.6},
		{442,	2614},
		{443,	2617.4},
		{444,	2620.8},
		{445,	2624.2},
		{446,	2627.6},
		{447,	2631},
		{448,	2634.3},
		{449,	2637.7},
		{450,	2641.1},
		{451,	2644.5},
		{452,	2647.9},
		{453,	2651.3},
		{454,	2654.7},
		{455,	2658},
		{456,	2661.4},
		{457,	2664.8},
		{458,	2668.2},
		{459,	2671.5},
		{460,	2674.9},
		{461,	2678.3},
		{462,	2681.7},
		{463,	2685},
		{464,	2688.4},
		{465,	2691.8},
		{466,	2695.1},
		{467,	2698.5},
		{468,	2701.9},
		{469,	2705.2},
		{470,	2708.6},
		{471,	2712},
		{472,	2715.3},
		{473,	2718.7},
		{474,	2722},
		{475,	2725.4},
		{476,	2728.8},
		{477,	2732.1},
		{478,	2735.5},
		{479,	2738.8},
		{480,	2742.2},
		{481,	2745.5},
		{482,	2748.9},
		{483,	2752.2},
		{484,	2755.6},
		{485,	2758.9},
		{486,	2762.3},
		{487,	2765.6},
		{488,	2768.9},
		{489,	2772.3},
		{490,	2775.6},
		{491,	2779},
		{492,	2782.3},
		{493,	2785.6},
		{494,	2789},
		{495,	2792.3},
		{496,	2795.6},
		{497,	2799},
		{498,	2802.3},
		{499,	2805.6},
		{500,	2809}
};


#endif /* INC_NTC_LOOKUP_TABLE_H_ */
