#include <stdlib.h>

class InfiniTAMApp {
	public:
	static InfiniTAMApp* Instance(void)
	{
		if (globalInstance==NULL) globalInstance = new InfiniTAMApp();
		return globalInstance;
	}

	void InitGL();
	void ResizeGL(int newWidth, int newHeight);
	void RenderGL(void);

	private:
	static InfiniTAMApp *globalInstance;
};


