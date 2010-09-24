#include<stdio.h>
#include<stdlib.h>
#include<string.h>

int max( int a, int b )
{
  if ( a > b )
    return a;
  else
    return b;
}

int min( int a, int b )
{
  if ( a < b )
    return a;
  else
    return b;
}

int main( int argc, char ** argv )
{
  FILE * in, * pos, * obj, * error, * out, *script;
  int count = 0;
  int maxy = 0;
  int miny = 0;
  int vpos, vobj, verror, vout;
  char tmp[80];

  if ( argc == 1 )
    return 0;

  in = fopen( argv[1], "r" );
  if ( in == NULL )
  {
    printf( "Error: Could not open in file\n" );
    return 0;
  }

  pos =  fopen( "pos.txt", "w" );
  obj = fopen( "obj.txt", "w" );
  error = fopen( "error.txt", "w" );
  out = fopen( "out.txt", "w" );
  script = fopen( "script.txt", "w" );

  while ( ! feof( in ) )
  {
    // fgets( tmp, 80, in );
    fscanf( in, "%d %d %d %d", &vobj, &vpos, &verror, &vout );
    fprintf( pos, "%d %d\n", count, vpos );
    fprintf( obj, "%d %d\n", count, vobj );
    fprintf( out, "%d %d\n", count, vout );
    fprintf( error, "%d %d\n", count, verror );
    count ++;
    maxy = max( maxy, max( vobj, max( vpos, max( vout, verror ) ) ) );
    miny = min( miny, min( vobj, min( vpos, min( vout, verror ) ) ) );
  }

  maxy += 100;
  miny -= 100;

  fprintf( script, "set term post\n" );
  fprintf( script, "set output \"test.ps\"\n" );
  fprintf( script, "#set autoscale                          # scale axes automatically\n" );
  fprintf( script, "unset log                              # remove any log-scaling\n" );
  fprintf( script, "unset label                            # remove any previous labels\n" );
  fprintf( script, "set xtic auto                          # set xtics\n" );
  fprintf( script, "set ytic auto\n" );
  fprintf( script, "set xrange [-5:%d];", count );
  fprintf( script, "set yrange [%d:%d];", miny, maxy );
  fprintf( script, "set multiplot;\n" );
  fprintf( script, "set origin 0.0,0.0;\n" );
  fprintf( script, "plot \"out.txt\" with lines lt 1 linecolor rgb \"red\";\n" );
  fprintf( script, "set origin 0.0,0.0;\n" );
  fprintf( script, "replot \"error.txt\" with lines lt 1 linecolor rgb \"blue\";\n" );
  fprintf( script, "set origin 0.0,0.0;\n" );
  fprintf( script, "replot \"obj.txt\" with lines lt 1 linecolor rgb \"black\";\n" );
  fprintf( script, "set origin 0.0,0.0;\n" );
  fprintf( script, "replot \"pos.txt\" with lines lt 1 lw 2 linecolor rgb \"dark-green\";\n" );
  fprintf( script, "unset multiplot;\n" );


  fclose( in );
  fclose( out );
  fclose( obj );
  fclose( error );
  fclose( pos );
  fclose( script );
  // rewind( f );

  return 0;
}

