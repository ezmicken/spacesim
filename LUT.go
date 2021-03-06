package spacesim

import(
  "github.com/ezmicken/fixpoint"
)

var LUTAccel []fixpoint.Vec3Q16 = []fixpoint.Vec3Q16{
  fixpoint.Vec3Q16FromFloat(0.000000, 1.000000, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.017452, 0.999848, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.034900, 0.999391, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.052336, 0.998630, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.069756, 0.997564, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.087156, 0.996195, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.104529, 0.994522, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.121869, 0.992546, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.139173, 0.990268, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.156435, 0.987688, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.173648, 0.984808, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.190809, 0.981627, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.207912, 0.978148, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.224951, 0.974370, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.241922, 0.970296, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.258819, 0.965926, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.275637, 0.961262, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.292372, 0.956305, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.309017, 0.951057, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.325568, 0.945519, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.342020, 0.939693, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.358368, 0.933580, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.374607, 0.927184, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.390731, 0.920505, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.406737, 0.913545, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.422618, 0.906308, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.438371, 0.898794, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.453991, 0.891007, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.469472, 0.882948, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.484810, 0.874620, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.500000, 0.866025, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.515038, 0.857167, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.529919, 0.848048, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.544639, 0.838671, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.559193, 0.829038, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.573577, 0.819152, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.587785, 0.809017, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.601815, 0.798636, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.615661, 0.788011, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.629320, 0.777146, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.642788, 0.766045, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.656059, 0.754710, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.669131, 0.743145, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.681998, 0.731354, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.694658, 0.719340, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.707107, 0.707107, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.719340, 0.694658, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.731354, 0.681998, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.743145, 0.669131, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.754710, 0.656059, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.766044, 0.642788, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.777146, 0.629320, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.788011, 0.615661, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.798636, 0.601815, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.809017, 0.587785, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.819152, 0.573577, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.829038, 0.559193, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.838671, 0.544639, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.848048, 0.529919, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.857167, 0.515038, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.866025, 0.500000, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.874620, 0.484810, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.882948, 0.469472, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.891007, 0.453991, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.898794, 0.438371, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.906308, 0.422618, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.913545, 0.406737, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.920505, 0.390731, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.927184, 0.374607, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.933580, 0.358368, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.939693, 0.342020, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.945519, 0.325568, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.951057, 0.309017, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.956305, 0.292372, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.961262, 0.275637, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.965926, 0.258819, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.970296, 0.241922, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.974370, 0.224951, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.978148, 0.207912, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.981627, 0.190809, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.984808, 0.173648, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.987688, 0.156434, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.990268, 0.139173, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.992546, 0.121869, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.994522, 0.104528, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.996195, 0.087156, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.997564, 0.069756, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.998630, 0.052336, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.999391, 0.034899, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.999848, 0.017452, 0.000000),
  fixpoint.Vec3Q16FromFloat(-1.000000, 0.000000, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.999848, -0.017452, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.999391, -0.034899, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.998630, -0.052336, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.997564, -0.069756, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.996195, -0.087156, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.994522, -0.104529, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.992546, -0.121869, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.990268, -0.139173, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.987688, -0.156434, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.984808, -0.173648, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.981627, -0.190809, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.978148, -0.207912, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.974370, -0.224951, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.970296, -0.241922, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.965926, -0.258819, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.961262, -0.275638, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.956305, -0.292372, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.951057, -0.309017, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.945519, -0.325568, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.939693, -0.342020, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.933581, -0.358368, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.927184, -0.374607, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.920505, -0.390731, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.913546, -0.406737, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.906308, -0.422618, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.898794, -0.438371, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.891007, -0.453991, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.882948, -0.469472, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.874620, -0.484810, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.866025, -0.500000, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.857167, -0.515038, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.848048, -0.529919, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.838671, -0.544639, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.829038, -0.559193, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.819152, -0.573577, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.809017, -0.587785, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.798636, -0.601815, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.788011, -0.615662, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.777146, -0.629320, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.766044, -0.642788, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.754710, -0.656059, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.743145, -0.669131, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.731354, -0.681998, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.719340, -0.694658, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.707107, -0.707107, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.694658, -0.719340, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.681998, -0.731354, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.669131, -0.743145, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.656059, -0.754710, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.642788, -0.766044, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.629320, -0.777146, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.615662, -0.788011, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.601815, -0.798636, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.587785, -0.809017, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.573577, -0.819152, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.559193, -0.829038, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.544639, -0.838671, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.529919, -0.848048, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.515038, -0.857167, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.500000, -0.866025, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.484810, -0.874620, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.469472, -0.882948, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.453991, -0.891007, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.438371, -0.898794, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.422618, -0.906308, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.406737, -0.913545, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.390731, -0.920505, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.374607, -0.927184, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.358368, -0.933580, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.342020, -0.939693, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.325568, -0.945519, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.309017, -0.951057, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.292372, -0.956305, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.275637, -0.961262, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.258819, -0.965926, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.241922, -0.970296, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.224951, -0.974370, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.207912, -0.978148, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.190809, -0.981627, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.173648, -0.984808, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.156435, -0.987688, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.139173, -0.990268, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.121869, -0.992546, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.104529, -0.994522, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.087156, -0.996195, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.069756, -0.997564, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.052336, -0.998630, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.034899, -0.999391, 0.000000),
  fixpoint.Vec3Q16FromFloat(-0.017452, -0.999848, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.000000, -1.000000, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.017452, -0.999848, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.034899, -0.999391, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.052336, -0.998630, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.069756, -0.997564, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.087156, -0.996195, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.104528, -0.994522, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.121869, -0.992546, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.139173, -0.990268, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.156434, -0.987688, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.173648, -0.984808, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.190809, -0.981627, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.207912, -0.978148, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.224951, -0.974370, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.241922, -0.970296, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.258819, -0.965926, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.275637, -0.961262, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.292372, -0.956305, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.309017, -0.951057, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.325568, -0.945519, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.342020, -0.939693, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.358368, -0.933581, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.374607, -0.927184, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.390731, -0.920505, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.406737, -0.913546, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.422618, -0.906308, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.438371, -0.898794, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.453991, -0.891007, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.469472, -0.882948, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.484810, -0.874620, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.500000, -0.866025, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.515038, -0.857167, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.529919, -0.848048, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.544639, -0.838671, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.559193, -0.829038, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.573577, -0.819152, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.587785, -0.809017, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.601815, -0.798636, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.615661, -0.788011, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.629320, -0.777146, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.642788, -0.766044, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.656059, -0.754710, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.669131, -0.743145, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.681998, -0.731354, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.694658, -0.719340, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.707107, -0.707107, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.719340, -0.694658, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.731354, -0.681998, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.743145, -0.669131, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.754710, -0.656059, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.766045, -0.642788, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.777146, -0.629320, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.788011, -0.615662, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.798635, -0.601815, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.809017, -0.587786, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.819152, -0.573576, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.829038, -0.559193, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.838671, -0.544639, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.848048, -0.529919, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.857167, -0.515038, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.866025, -0.500000, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.874620, -0.484810, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.882948, -0.469472, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.891007, -0.453991, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.898794, -0.438371, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.906308, -0.422618, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.913546, -0.406737, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.920505, -0.390731, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.927184, -0.374607, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.933581, -0.358368, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.939693, -0.342020, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.945519, -0.325568, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.951057, -0.309017, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.956305, -0.292372, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.961262, -0.275637, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.965926, -0.258819, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.970296, -0.241922, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.974370, -0.224951, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.978148, -0.207912, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.981627, -0.190809, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.984808, -0.173648, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.987688, -0.156434, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.990268, -0.139173, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.992546, -0.121870, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.994522, -0.104528, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.996195, -0.087156, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.997564, -0.069756, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.998630, -0.052336, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.999391, -0.034900, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.999848, -0.017452, 0.000000),
  fixpoint.Vec3Q16FromFloat(1.000000, 0.000000, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.999848, 0.017452, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.999391, 0.034899, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.998630, 0.052336, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.997564, 0.069757, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.996195, 0.087156, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.994522, 0.104528, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.992546, 0.121869, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.990268, 0.139173, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.987688, 0.156434, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.984808, 0.173648, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.981627, 0.190809, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.978148, 0.207911, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.974370, 0.224951, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.970296, 0.241922, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.965926, 0.258819, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.961262, 0.275637, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.956305, 0.292372, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.951057, 0.309017, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.945519, 0.325568, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.939693, 0.342020, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.933581, 0.358368, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.927184, 0.374606, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.920505, 0.390731, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.913545, 0.406737, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.906308, 0.422618, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.898794, 0.438371, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.891007, 0.453990, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.882948, 0.469472, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.874620, 0.484810, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.866026, 0.500000, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.857167, 0.515038, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.848048, 0.529919, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.838671, 0.544639, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.829038, 0.559193, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.819152, 0.573576, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.809017, 0.587785, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.798635, 0.601815, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.788011, 0.615662, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.777146, 0.629320, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.766045, 0.642788, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.754710, 0.656059, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.743145, 0.669131, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.731354, 0.681998, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.719340, 0.694658, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.707107, 0.707107, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.694659, 0.719340, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.681998, 0.731354, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.669131, 0.743145, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.656059, 0.754710, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.642788, 0.766044, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.629321, 0.777146, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.615661, 0.788011, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.601815, 0.798636, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.587785, 0.809017, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.573577, 0.819152, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.559193, 0.829037, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.544639, 0.838671, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.529919, 0.848048, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.515038, 0.857167, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.500000, 0.866025, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.484810, 0.874620, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.469472, 0.882948, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.453991, 0.891007, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.438371, 0.898794, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.422619, 0.906308, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.406737, 0.913546, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.390731, 0.920505, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.374607, 0.927184, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.358368, 0.933580, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.342020, 0.939693, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.325568, 0.945519, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.309017, 0.951057, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.292372, 0.956305, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.275638, 0.961262, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.258819, 0.965926, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.241922, 0.970296, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.224951, 0.974370, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.207912, 0.978148, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.190809, 0.981627, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.173648, 0.984808, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.156434, 0.987688, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.139173, 0.990268, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.121869, 0.992546, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.104529, 0.994522, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.087156, 0.996195, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.069756, 0.997564, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.052336, 0.998630, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.034900, 0.999391, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.017453, 0.999848, 0.000000),
  fixpoint.Vec3Q16FromFloat(0.000000, 1.000000, 0.000000),
}
