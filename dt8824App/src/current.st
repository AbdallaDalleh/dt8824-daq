program current

double Raw;
assign Raw to "{SYSTEM}:getVoltage";

double Current;
assign Current to "{SYSTEM}:getCurrent";

int Gain;
assign Gain to "{SYSTEM}:setSensitivity";

double AverageVoltage;
assign AverageVoltage to "{SYSTEM}:getAverageVoltage";
monitor AverageVoltage;

double AverageCurrent;
assign AverageCurrent to "{SYSTEM}:getAverageCurrent";


double gains[28] = {0.000000000001, 0.000000000002, 0.000000000005, 0.00000000001, 0.00000000002, 0.00000000005, 0.0000000001, 0.0000000002, 0.0000000005, 0.000000001, 0.000000002, 0.000000005, 0.00000001, 0.00000002, 0.00000005, 0.0000001, 0.0000002, 0.0000005, 0.000001, 0.000002, 0.000005, 0.00001, 0.00002, 0.00005, 0.0001, 0.0002, 0.0005, 0.001};

ss Current
{	
	state Main
	{
		entry
		{
			// pvGet(Raw);
			// pvGet(Gain);
			// pvGet(AverageVoltage);
		}			
		when(delay(2))
		{	
			pvGet(Raw);
			pvGet(Gain);
			// printf("Raw: %f\n", Raw);
			// printf("Gain: %d\n", Gain);
			Current = gains[Gain] * Raw;		
			pvPut(Current);
			AverageCurrent = gains[Gain] * AverageVoltage;
			pvPut(AverageCurrent);
			// printf("Current: %E\n", Current);
			// printf("================\n");
		} state Main
	}
}
