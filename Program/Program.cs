using System.Collections.Generic;
using System;
using System.Linq;

namespace NikonControl
{
    class Program
    {
        public static int[] intParams;
        public static string imgName;
        
        //Modes:
        //0 - single image of specified aperture and shutter indices
        //1 - multiple images starting from specified aperture and at given shutter speed

        //arguments: [Mode] [number of Apertures] [shutter speed] 

        static void Main(string[] args)
        {
            intParams = args.Select(x => int.Parse(x)).ToArray();
            Aux aux = new Aux();

            Console.WriteLine(intParams[1]);
            //.WriteLine(DateTime.Now.ToString("hh.mm.ss.fffffff"));
            switch (intParams[0])
            {
                case 0:                         //1 aperture and shutterspeed
                    aux.Single(intParams[1], intParams[2]);
                    break;
                case 1:                         //# apertures at fixed shutterspeed
                    aux.Multiple(intParams[1], intParams[2]);
                    break;
                case 2:                         //# images at Fstop
                    aux.FStop(intParams[1]);
                    break;
                case 3:                         //#images with shutter decreasing from s 
                    aux.MultipleExp(intParams[1], intParams[2]);
                    break;
            }
        }
    }




}
