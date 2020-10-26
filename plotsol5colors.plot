# define a palette with an exact number of colors
set palette maxcolors 5
set palette model RGB defined ( \
  0 "red", \
  1 "blue", \
  2 "green", \
  3 "orange", \
  4 "cyan")
 
 # Define palette labels, the range matches the number of colors defined above
#set cbrange [0:5]
#set cbtics offset 0,+5 ( \
#  'Group 1' 0, \
#  'Group 2' 1, \
#  'Group 3' 2, \
#  'Group 4' 3, \
#  'Group 5' 4, \
#  5)

unset key;
unset grid;
set tics scale 0;
set format '';
unset border;
unset colorbox;
#plot 'bus_32_1_edges.dat' using 1:2:3 with lines linestyle 1 lc palette, 'bus_32_1_vertices.dat' with points pt 7 lc 'black'
#plot 'bus_32_1_edges.dat' using 1:2:3 with lines linestyle 1 lc palette, 'bus_32_1_edges.dat' using 1:2 with points pt 7 lc 'black'
#plot 'JaboticabalJAB05_edges.dat' using 1:2:3 with lines linestyle 1 lc palette, 'JaboticabalJAB05_edges.dat' using 1:2 with dots lc 'black'
plot 'jaboticabal3_5g.txt' using 1:2:3 with lines linestyle 1 lc palette, 'jaboticabal3_5g.txt' using 1:2 with dots lc 'black',  '-' with points pt 7 ps 0.5 lc 'black'
779160 649284
