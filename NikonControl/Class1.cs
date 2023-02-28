//
// This work is licensed under a Creative Commons Attribution 3.0 Unported License.
//
// Thomas Dideriksen (thomas@dideriksen.com)
//

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;
using Nikon;
using System.Globalization;

//
// This demo shows how to capture an image and store it to disk
//

namespace NikonControl
{
    
    

    public class Aux
    {
        NikonDevice _device;
        AutoResetEvent _waitForDevice = new AutoResetEvent(false);
        AutoResetEvent _waitForCaptureComplete = new AutoResetEvent(false);
        uint _captureLimit = 20;
        uint _captureCount = 0;
        string imgName;

        int[] fstop_a = { 0, 4, 7, 9, 12, 15, 18, 19 }; //apertures
        int[] fstop_s = { 33, 31, 29, 28, 27, 25, 23, 21}; //shutters




        public void FStop(int count)
        {


            try
            {
                // Create manager object - make sure you have the correct MD3 file for the Z7 (type 23)
                NikonManager manager = new NikonManager("Type0023.md3");

                // Listen for the 'DeviceAdded' event
                manager.DeviceAdded += manager_DeviceAdded;

                // Wait for a device to arrive
                Console.WriteLine("Hello there we got this far!");
                _waitForDevice.WaitOne();
                Console.WriteLine("Found one device");

                // Set shooting mode to 'continuous, highspeed'
                //NikonEnum shootingMode = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_ShootingMode);
                //shootingMode.Index = (int)eNkMAIDShootingMode.kNkMAIDShootingMode_CH;
                //_device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_ShootingMode, shootingMode);

                _device.SetUnsigned(eNkMAIDCapability.kNkMAIDCapability_ContinuousShootingNum, 1);

                // Hook up capture events
                _device.SetUnsigned(eNkMAIDCapability.kNkMAIDCapability_SaveMedia, 1);
                _device.ImageReady += _device_ImageReady;
                _device.CaptureComplete += _device_CaptureComplete;

                //change aperture
                NikonEnum apertureMode = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_Aperture);

                //change shutter speed
                NikonEnum shutterSpeed = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_ShutterSpeed);
                

                //printing out the available shutter speeds
                //for (int i = 0; i < shutterSpeed.Length; i++)
                //{
                //   Console.WriteLine(shutterSpeed.GetEnumValueByIndex(i).ToString());
                //}


                //printing out the available apertures
                for (int i = 0; i < count; i++)
                {
                    int s = fstop_s[i];
                    int a = fstop_a[i];

                    //Console.WriteLine(apertureMode.GetEnumValueByIndex(i).ToString());
                    apertureMode.Index = a;
                    Console.WriteLine("Aperture: " + apertureMode.GetEnumValueByIndex(a).ToString());
                    _device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_Aperture, apertureMode);

                    

                    shutterSpeed.Index = s;
                    Console.Write("Shutter Speed: ");
                    Console.WriteLine(shutterSpeed[s]);
                    _device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_ShutterSpeed, shutterSpeed);

                    imgName = "Fa" + i + "_s" + s;

                    // Capture
                    _device.Capture();

                    // Wait for the capture to complete
                    _waitForCaptureComplete.WaitOne();
                    Console.WriteLine("Captured one image");
                    // Shutdown

                }

                manager.Shutdown();



            }
            catch (NikonException ex)
            {
                Console.WriteLine(ex.Message);
            }
        }

        public void MultipleExp(int a, int s)
        {


            try
            {
                // Create manager object - make sure you have the correct MD3 file for the Z7 (type 23)
                NikonManager manager = new NikonManager("Type0023.md3");

                // Listen for the 'DeviceAdded' event
                manager.DeviceAdded += manager_DeviceAdded;

                // Wait for a device to arrive
                Console.WriteLine("Hello there we got this far!");
                _waitForDevice.WaitOne();
                Console.WriteLine("Found one device");

                // Set shooting mode to 'continuous, highspeed'
                //NikonEnum shootingMode = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_ShootingMode);
                //shootingMode.Index = (int)eNkMAIDShootingMode.kNkMAIDShootingMode_CH;
                //_device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_ShootingMode, shootingMode);

                _device.SetUnsigned(eNkMAIDCapability.kNkMAIDCapability_ContinuousShootingNum, 1);

                // Hook up capture events
                _device.SetUnsigned(eNkMAIDCapability.kNkMAIDCapability_SaveMedia, 1);
                _device.ImageReady += _device_ImageReady;
                _device.CaptureComplete += _device_CaptureComplete;

                //change aperture
                NikonEnum apertureMode = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_Aperture);
                apertureMode.Index = 14;
                Console.WriteLine("Aperture: " + apertureMode.GetEnumValueByIndex(14).ToString());
                _device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_Aperture, apertureMode);

                //change shutter speed
                NikonEnum shutterSpeed = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_ShutterSpeed);
                shutterSpeed.Index = 6;
                Console.Write("Shutter Speed: ");
                Console.WriteLine(shutterSpeed[6]);
                _device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_ShutterSpeed, shutterSpeed);

                imgName = "EHomography";

                _device.Capture();

                // Wait for the capture to complete
                _waitForCaptureComplete.WaitOne();
                Console.WriteLine("Captured homography image");

                //printing out the available shutter speeds
                //for (int i = 0; i < shutterSpeed.Length; i++)
                //{
                //   Console.WriteLine(shutterSpeed.GetEnumValueByIndex(i).ToString());
                //}


                //printing out the available apertures
                for (int i = 0; i < a; i++)
                {
                    //Console.WriteLine(apertureMode.GetEnumValueByIndex(i).ToString());
                    apertureMode.Index = i;
                    Console.WriteLine("Aperture: " + apertureMode.GetEnumValueByIndex(i).ToString());
                    _device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_Aperture, apertureMode);

                    float weight = (float)i / (float)a;
                    int z = (int)(MathF.Exp(4f * weight - 4f) * 19);
                    //Console.WriteLine(i + " " + a);
                    //Console.WriteLine("weight: " + weight);
                    Console.WriteLine("setting shutterspeed to: " + (s - z));
                    imgName = "Ea" + i + "_s" + (s - z);

                    shutterSpeed.Index = s-z;
                    Console.Write("Shutter Speed: ");
                    Console.WriteLine(shutterSpeed[s-z]);
                    _device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_ShutterSpeed, shutterSpeed);
                    // Capture
                    _device.Capture();

                    // Wait for the capture to complete
                    _waitForCaptureComplete.WaitOne();
                    Console.WriteLine("Captured one image");
                    // Shutdown

                }

                manager.Shutdown();



            }
            catch (NikonException ex)
            {
                Console.WriteLine(ex.Message);
            }
        }

        public void Multiple(int a, int s)
        {


            try
            {
                // Create manager object - make sure you have the correct MD3 file for the Z7 (type 23)
                NikonManager manager = new NikonManager("Type0023.md3");

                // Listen for the 'DeviceAdded' event
                manager.DeviceAdded += manager_DeviceAdded;

                // Wait for a device to arrive
                Console.WriteLine("Hello there we got this far!");
                _waitForDevice.WaitOne();
                Console.WriteLine("Found one device");

                // Set shooting mode to 'continuous, highspeed'
                //NikonEnum shootingMode = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_ShootingMode);
                //shootingMode.Index = (int)eNkMAIDShootingMode.kNkMAIDShootingMode_CH;
                //_device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_ShootingMode, shootingMode);

                _device.SetUnsigned(eNkMAIDCapability.kNkMAIDCapability_ContinuousShootingNum, 1);

                // Hook up capture events
                _device.SetUnsigned(eNkMAIDCapability.kNkMAIDCapability_SaveMedia, 1);
                _device.ImageReady += _device_ImageReady;
                _device.CaptureComplete += _device_CaptureComplete;

                //change aperture
                NikonEnum apertureMode = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_Aperture);

                //change shutter speed
                NikonEnum shutterSpeed = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_ShutterSpeed);
                shutterSpeed.Index = s;
                Console.Write("Shutter Speed: ");
                Console.WriteLine(shutterSpeed[s]);
                _device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_ShutterSpeed, shutterSpeed);

                //printing out the available shutter speeds
                //for (int i = 0; i < shutterSpeed.Length; i++)
                //{
                //   Console.WriteLine(shutterSpeed.GetEnumValueByIndex(i).ToString());
                //}


                //printing out the available apertures
                for (int i = 0; i < a; i++)
                {
                    //Console.WriteLine(apertureMode.GetEnumValueByIndex(i).ToString());
                    apertureMode.Index = i;
                    Console.WriteLine("Aperture: " + apertureMode.GetEnumValueByIndex(i).ToString());
                    _device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_Aperture, apertureMode);

                    imgName = "a" + i + "_s" + s;

                    // Capture
                    _device.Capture();

                    // Wait for the capture to complete
                    _waitForCaptureComplete.WaitOne();
                    Console.WriteLine("Captured one image");
                    // Shutdown
                    
                }

                manager.Shutdown();



            }
            catch (NikonException ex)
            {
                Console.WriteLine(ex.Message);
            }
        }

        //arguments: aperture, shutter
        public void Single(int a, int s)
        {
            

            try
            {
                // Create manager object - make sure you have the correct MD3 file for the Z7 (type 23)
                NikonManager manager = new NikonManager("Type0023.md3");

                // Listen for the 'DeviceAdded' event
                manager.DeviceAdded += manager_DeviceAdded;

                // Wait for a device to arrive
                Console.WriteLine("Hello there we got this far!");
                _waitForDevice.WaitOne();
                Console.WriteLine("Found one device");

                // Set shooting mode to 'continuous, highspeed'
                //NikonEnum shootingMode = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_ShootingMode);
                //shootingMode.Index = (int)eNkMAIDShootingMode.kNkMAIDShootingMode_CH;
                //_device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_ShootingMode, shootingMode);

                _device.SetUnsigned(eNkMAIDCapability.kNkMAIDCapability_ContinuousShootingNum, 1);

                // Hook up capture events
                _device.SetUnsigned(eNkMAIDCapability.kNkMAIDCapability_SaveMedia, 1);
                _device.ImageReady += _device_ImageReady;
                _device.CaptureComplete += _device_CaptureComplete;

                //change aperture
                NikonEnum apertureMode = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_Aperture);
                apertureMode.Index = a;
                Console.WriteLine(apertureMode[a]);
                _device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_Aperture, apertureMode);

                //change shutter speed
                NikonEnum shutterSpeed = _device.GetEnum(eNkMAIDCapability.kNkMAIDCapability_ShutterSpeed);
                shutterSpeed.Index = s;

                imgName = "a" + a + "_s" + s;
                //printing out the available shutter speeds
                //for (int i = 0; i < shutterSpeed.Length; i++)
                //{
                //   Console.WriteLine(shutterSpeed.GetEnumValueByIndex(i).ToString());
                //}

                //printing out the available apertures
                //for (int i = 0; i < apertureMode.Length; i++)
                //{
                 //   Console.WriteLine(apertureMode.GetEnumValueByIndex(i).ToString());
                //}

                Console.WriteLine(shutterSpeed[s]);
                _device.SetEnum(eNkMAIDCapability.kNkMAIDCapability_ShutterSpeed, shutterSpeed);

                // Capture
                _device.Capture();

                // Wait for the capture to complete
                _waitForCaptureComplete.WaitOne();
                Console.WriteLine("Captured one image");
                // Shutdown
                manager.Shutdown();
            }
            catch (NikonException ex)
            {
                Console.WriteLine(ex.Message);
            }
        }

        void _device_ImageReady(NikonDevice sender, NikonImage image)
        {
            // Save captured image to disk
            var format = new CultureInfo("de-DE");
            string filename = imgName + ((image.Type == NikonImageType.Jpeg) ? ".jpg" : ".nef");
            _captureCount++;

            using (FileStream s = new FileStream(filename, FileMode.Create, FileAccess.Write))
            {
                s.Write(image.Buffer, 0, image.Buffer.Length);
            }
        }

        void _device_CaptureComplete(NikonDevice sender, int data)
        {
            // Signal the the capture completed
            _waitForCaptureComplete.Set();
        }

        void manager_DeviceAdded(NikonManager sender, NikonDevice device)
        {
            if (_device == null)
            {
                // Save device
                _device = device;

                // Signal that we got a device
                _waitForDevice.Set();
            }
        }
    }

}
