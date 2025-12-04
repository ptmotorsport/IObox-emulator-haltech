# IO Box Emulator (Emtron Compatible)
This is an arduino project that will emulate the functionality of an Emtron EIC16M CAN expander.

The original code was published by tolunaygul, thanks to him for doing the heavy lifting with this code!

Thank you for your support! We greatly appreciate you spreading the word about our products and services to your fellow motorsport enthusiasts.

Projects like these are made possible by your support through purchases on the PT Motorsport AU website. 

www.ptmotorsport.com.au

Completed arduino CAN expanders are avalibale on the PT Motorsport AU website in a variety of form factors and price points.

https://www.ptmotorsport.com.au/product-tag/pt-motorsport-io-expander/

# ECU Setup
The Emtron EIC16M has 16 Analog inputs, four of which can be changed to display frequencies.

The PT Motorsport IO expanders are designed around 4 analog inputs, 4 digital inputs, and 4 pulsed outputs.

The functionality of the first four analog inputs remains the same, with 0-5V being sent to the Emtron and seen as a specific voltage input.

Due to the PT Motorsport IO having four digital inputs, the signals sent to the Emtron will display as analog inputs 5-8 in the ECU, but will switch from 0V straight to 5V depending on the switch state. Digital Input 1 on the IO expander can still read frequency messages, and these are sent to the Emtron as well.

Unlike the Emtron EIC16M, the PT Motorsport IO expanders also have 4 output channels, these can also be made to work with the Emtron ECUs. This functionality is achieved within the EMTune program, and details of that can be found in the link below.

A full write up of how to use this firmware and ECU setup can be found here:

https://www.ptmotorsport.com.au/emtron-can-information-for-use-with-pt-motorsport-io-expanders

# Supporting hardware
You'll need a little more than an arduino Uno or Arduino Nano to make this into a reliable and functional bit of equipment.
All the hardware is avaliable on the PT Motorsport AU wesbite

https://www.ptmotorsport.com.au/product-tag/pt-motorsport-io-expander/

# Firmware Updates
To update the firmware we have created this guide

https://www.ptmotorsport.com.au/wp-content/uploads/2025/03/IO-Expander-Firmware-update-Instruction-Manual.pdf

# Licence
Copyright (c) 2023 - PT Motorsport AU Pty Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
