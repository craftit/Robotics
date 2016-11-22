
#include "Ravl/Option.hh"
#include "Ravl/ParseCSV.hh"
#include "Ravl/Sums1d2.hh"
#include "Ravl/SysLog.hh"

using RavlN::StringC;

int main(int nargs,char **argv)
{
  RavlN::OptionC opt(nargs,argv);

  StringC str = opt.String("i","runlog.csv","Input data file.");

  opt.Check();

  RavlN::ParseCSVC parseData(0,false,"\t");

  parseData.Open(str);

  RavlN::SArray1dC<StringC> values;
  RavlN::Sums1d2C sumx;
  RavlN::Sums1d2C sumy;
  while(parseData.ReadValues(values)) {
    //RavlDebug("Values:%d ",(int) values.Size());
    sumx += values[1].IntValue();
    sumy += values[2].IntValue();


  }
  RavlInfo(" SumX:%f (%f) SumY:%f (%f) ",sumx.Mean(),RavlN::Sqrt(sumx.Variance()),sumy.Mean(),RavlN::Sqrt(sumy.Variance()));
  return 0;
}
