#ifndef LINE_SEGGER_XXH
#define LINE_SEGGER_XXH

void SegmentScanLine (int scanno);
bool getOnelist (int *startid, int *endid);
void segment(int start_in, int finish_in, double *sig_out, double *sum_out);
void transform (int start,int finish);
void deviation (int *pos, double *dev, int *ok, double *sum);
void improve_lines (int start_in,int finish_in);
void LineExtraction (int scanno);
bool LineSegger ();

#endif
