# Driver for MIDI concertina running on Adafruit's Bluefruit 32u4.

The concertina's keyboard consists of a 14-column by 4-row matrix of buttons. Each button 
consists of a normally open pushbutton switch in series with a diode. The anode of the 
diode is connected to the button's row. Its cathode is connected to the switch. The other 
end of the switch is connected to the button's column. Each row is connected to a digital 
input pin on the Bluefruit set to INPUT_PULLUP mode. Each column is connected to one of the 
output pins on one of two HC164 8-bit shift registers (of which only 7 bits are used). The 
HC164s are connected in series, effectively making them into a 14-bit shift register. One 
HC164 is used for the 28 buttons on the left side of the concertina  and the other for the 
28 buttons on the concertina's right side.

When a columns's HC164 output pin is HIGH, the column is deselected. That is, since its 
row (like all the rows) is in INPUT_PULLUP mode, there is no voltage difference between 
the anodes of the column's buttons and the column's output pin. That means pushing any 
of them has no effect. On the other hand, setting a column's HC164 output pin to LOW 
selects it. This is because doing so creates a voltage difference between column's 
buttons' anodes and the far ends of each of the switches. If a button's switch is pushed, 
that button's row is pulled to ground and can be detected by the Bluefruit.

On each trip through loop() the sketch processes the buttons in one column -- the selected 
column. Column selection is done as follows. When it is time to sample the first column, 
the sketch injects a LOW into the input of the HC164 for the left side of the concertina 
and strobes the HC164's clock input. This selects the first column. On subsequent trips 
through loop(), a HIGH is injected into the HC164's input and the clock is strobed. This
shifts the LOW one column farther along, selecting it. After 14 trips through loop(), the 
whole process starts again with the first column.

Once the column for this trip through loop() has been selected, the state of each button
in that column is sampled: If the input pin corresponding to a button is LOW, the button
is down. If it's HIGH, the button is up.

The sketch keeps track of the state of the buttons as they are scanned. If a button's 
state changes and the Bluefruit's BLE radio is connected (presumably to a MIDI synth), the 
sketch uses the Adafruit BLE library to send the appropriate MIDI message using the radio.

The concertina also contains a differential pressure sensor that measures the difference 
between the pressure on the inside and outside of the bellows. It is read as an analog value 
on pin P_SENSE. The absoulte value of the pressure difference between the inside and outside 
of the bellows tells how hard the bellows is being worked. This value is used to to set the 
MIDI expression pedal (instrument volume) value via a pressure-to-expression transfer 
function. Approximately every EXP_MSG_MS milliseconds the pressure is read, transformed 
to an expression value and sent via MIDI as an expression pedal value.
