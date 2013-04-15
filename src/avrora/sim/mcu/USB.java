package avrora.sim.mcu;

import avrora.sim.RWRegister;
import avrora.sim.state.RegisterUtil;
import avrora.sim.state.RegisterView;
import avrora.sim.state.RegisterView.RegisterValueSetListener;

// Stub implementation of USB device
public class USB extends AtmelInternalDevice {

	// TODO actual implementation
	
	RWRegister PLLCSR_reg;
	PLLCSR_Watcher PLLCSR_watcher;
	
	public USB(AtmelMicrocontroller m) {
		super("USB", m);
		// TODO Auto-generated constructor stub
		
		PLLCSR_reg = new RWRegister();
		PLLCSR_watcher = new PLLCSR_Watcher();
		PLLCSR_reg.registerValueSetListener(PLLCSR_watcher);
		
		installIOReg("PLLCSR", PLLCSR_reg);
	}

	protected class PLLCSR_Watcher implements RegisterValueSetListener {
		
		final RegisterView PLOCK = RegisterUtil.bitView(PLLCSR_reg, 0);
		final RegisterView PLLE = RegisterUtil.bitView(PLLCSR_reg, 1);
		
		int plle = -1; 
		
		@Override
		public void onValueSet(RegisterView view, int oldValue, int newValue) {
			int nplle = PLLE.getValue();
			if (nplle != plle)
			{
				plle = nplle;
				PLOCK.setValue(plle);

				if (devicePrinter != null) {
					devicePrinter.println("USB PLLE " + plle);
				}
			}
		}
	}
}
