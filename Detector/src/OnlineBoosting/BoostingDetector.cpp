#include "BoostingDetector.h"

using namespace Detector;
BoostingDetector::BoostingDetector(StrongClassifier* classifier)
{
	this->m_classifier = classifier;

	m_confidences = NULL;
	m_sizeConfidences = 0;
	m_maxConfidence = -FLOAT_MAX;
	m_numDetections = 0;
	m_idxDetections = NULL;
	m_idxBestDetection = -1;
	m_sizeDetections = 0;

  m_confMatrix = cvCreateMat(1,1,CV_32FC1);
	m_confMatrixSmooth = cvCreateMat(1,1,CV_32FC1);

}

BoostingDetector::~BoostingDetector()
{
	if (m_idxDetections != NULL)
		delete[] m_idxDetections;
	if (m_confidences != NULL)
		delete[] m_confidences;

  cvReleaseMat(&m_confMatrix);
  cvReleaseMat(&m_confMatrixSmooth);

}

void BoostingDetector::prepareConfidencesMemory(int numPatches)
{
	if ( numPatches <= m_sizeConfidences )	
		return;							
	
	if ( m_confidences )
		delete[] m_confidences;

	m_sizeConfidences = numPatches;
    m_confidences = new float[numPatches];
}

void BoostingDetector::prepareDetectionsMemory(int numDetections)
{
	if ( numDetections <= m_sizeDetections ){	
		//std::cout << "Imamo M-size: "<< m_sizeDetections << std::endl;
		return;				
	}			
	
	if ( m_idxDetections ){
		//std::cout << "Detector: delete " << std::endl;
		delete[] m_idxDetections;
		//std::cout << "Detector: deleted " << std::endl;
	}
	m_sizeDetections = numDetections;
	//std::cout << "Detector: nastavimo " << std::endl;
    m_idxDetections = new int[numDetections];
	//std::cout << "Detector: new int OK " << std::endl;
}


void BoostingDetector::classify(ImageRepresentation* image, Patches* patches, float minMargin)
{
	int numPatches = patches->getNum();

	prepareConfidencesMemory(numPatches);

	m_numDetections = 0;
	m_idxBestDetection = -1;
	m_maxConfidence = -FLOAT_MAX;
	int numBaseClassifiers = m_classifier->getNumBaseClassifier();

	for (int curPatch=0; curPatch < numPatches; curPatch++)
	{
		m_confidences[curPatch] = m_classifier->eval(image, patches->getRect(curPatch));
	
		if (m_confidences[curPatch] > m_maxConfidence)
		{
			m_maxConfidence = m_confidences[curPatch];
			m_idxBestDetection = curPatch;
		}
		if (m_confidences[curPatch] > minMargin)
			m_numDetections++;
	}

	prepareDetectionsMemory(m_numDetections);
	int curDetection = -1;
	for (int curPatch=0; curPatch < numPatches; curPatch++)
	{
		if (m_confidences[curPatch]>minMargin) m_idxDetections[++curDetection]=curPatch;
	}
}


void BoostingDetector::classifySmooth(ImageRepresentation* image, Patches* patches, float minMargin)
{
	int numPatches = patches->getNum();

	//std::cout << "Detector: 1 " << std::endl;
	prepareConfidencesMemory(numPatches);

	m_numDetections = 0;
	m_idxBestDetection = -1;
	m_maxConfidence = -FLOAT_MAX;
	//std::cout << "Detector: 2 " << std::endl;
	int numBaseClassifiers = m_classifier->getNumBaseClassifier();

	//std::cout << "Detector: 3 " << std::endl;
	PatchesRegularScan *regPatches = (PatchesRegularScan*)patches;
	//std::cout << "Detector: 4 " << std::endl;
	Size patchGrid = regPatches->getPatchGrid();

	//std::cout << "Detector: 5 " << std::endl;
  if((patchGrid.width != m_confMatrix->cols) || (patchGrid.height != m_confMatrix->rows)) {
    cvReleaseMat(&m_confMatrix);
    cvReleaseMat(&m_confMatrixSmooth);

		//std::cout << "Detector: 6 " << std::endl;
    m_confMatrix = cvCreateMat(patchGrid.height,patchGrid.width,CV_32FC1);
	  m_confMatrixSmooth = cvCreateMat(patchGrid.height,patchGrid.width,CV_32FC1);
  }
	
	int curPatch = 0;
	// Eval and filter
	//std::cout << "Detector: 7 "<< std::endl;
	for(int row = 0; row < patchGrid.height; row++) {
		for( int col = 0; col < patchGrid.width; col++) {
			int returnedInLayer;
			//std::cout << "Detector: 7.5 " << std::endl;
			m_confidences[curPatch] = m_classifier->eval(image, patches->getRect(curPatch)); 
		
			// fill matrix
			cvmSet(m_confMatrix,row,col,m_confidences[curPatch]);
			curPatch++;
		}
	}

  // Filter
	//std::cout << "Detector: 8 " << std::endl;
	cvSmooth(m_confMatrix,m_confMatrixSmooth,CV_GAUSSIAN,3);

	// Get best detection
	curPatch = 0;
	//std::cout << "Detector: 9 " << std::endl;
	for(int row = 0; row < patchGrid.height; row++) {
		for( int col = 0; col < patchGrid.width; col++) {
			// fill matrix
			m_confidences[curPatch] = cvmGet(m_confMatrixSmooth,row,col);

			if (m_confidences[curPatch] > m_maxConfidence)
			{
				m_maxConfidence = m_confidences[curPatch];
				m_idxBestDetection = curPatch;
			}
			if (m_confidences[curPatch] > minMargin) {
				m_numDetections++;
			}
			curPatch++;
		}
	}

	//std::cout << "Detector: 10 " << std::endl;
	//std::cout << "st det: "<< m_numDetections << std::endl;
	prepareDetectionsMemory(m_numDetections);
	//std::cout << "Detector: 10.5 " << std::endl;
	int curDetection = -1;
	for (int curPatch=0; curPatch < numPatches; curPatch++)
	{
		//std::cout << m_confidences[curPatch] << "Margin : "<< minMargin<< std::endl;
		if (m_confidences[curPatch] > minMargin){
			m_idxDetections[++curDetection]=curPatch;
			}
	}
	//std::cout << "Detector: 11 " << std::endl;
}


	
int BoostingDetector::getNumDetections()
{
	return m_numDetections; 
}

float BoostingDetector::getConfidence(int patchIdx)
{
	return m_confidences[patchIdx];
}

float BoostingDetector::getConfidenceOfDetection (int detectionIdx)
{
	return m_confidences[getPatchIdxOfDetection(detectionIdx)];
}

int BoostingDetector::getPatchIdxOfBestDetection()
{
	return m_idxBestDetection;
}

int BoostingDetector::getPatchIdxOfDetection(int detectionIdx)
{
	return m_idxDetections[detectionIdx];
}
