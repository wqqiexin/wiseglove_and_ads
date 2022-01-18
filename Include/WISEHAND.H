/*####################################################################################
$$WISEHAND SDK, Ver 1.3 beta
$$File name		:		WISEHAND.h
$$Compiler		:		VC6
$$Create date	:		2012-6 
$$Revise date	:		2012-7 
$$History		 	:		--
$$Auther		 	:	  
$$Powered By	:		Wonstar CHAN Tech. Co., Ltd.	<www.wonstar.com>
####################################################################################*/

#ifdef VIRTUALHAND_EXPORTS
#define VIRTUALHAND_API __declspec(dllexport)
#else
#define VIRTUALHAND_API __declspec(dllimport)
#endif

/*手模型标志，用于setVhandtype*/
#define VHAND_LEFT  1
#define VHAND_RIGHT  0

class VIRTUALHAND_API WiseHand {
	public:
	virtual bool loadModelData( const char *filename )=0;
	virtual void RenderJoints(int type)=0;
	virtual void drawhand(float *angle, float *quat, int mode)=0;
	virtual void setVhandtype( bool type )=0;
};

VIRTUALHAND_API WiseHand* GetWiseHand(void);


VIRTUALHAND_API void DelWiseHand(WiseHand* p);