#include "gb.hxx"
#include <atlbase.h>
#include <atlcoll.h>
#include <atltypes.h>
#include <tchar.h>
#include <include/xaudio2.h>

// input:directinput8/win32msg input 
// graphics:direct3d/win32gdi 
// audio:directsound/xaudio2/mme-api

#define ROM_PATH L"E://bgbtest.gb"
#define ROM_PATH L"E://99s.gb"
#define ROM_PATH L"E://BattleCity (J) [!].gb"


//#define ROM_PATH L"E://02-interrupts.gb"
//#define ROM_PATH L"E://99s.gb"
//#define ROM_PATH L"E://yybs4.gb"
//#define ROM_PATH L"E://8se.gb"
//#define ROM_PATH L"E://instr_timing.gb"
//#define ROM_PATH L"E://mem_timing.gb"
//#define ROM_PATH L"E://01-read_timing.gb"
// #define ROM_PATH L"E://02-write_timing.gb"
//#define ROM_PATH L"E://99s.gb"
#define ROM_PATH L"E://zelda_dx.gbc"
struct CCanvas
{
  LPVOID m_pBuffer;
  UINT_PTR m_nPitch; // bytes.
  CSize m_szBuffer;
};

struct CGdiBuffer : public CCanvas
{
  CGdiBuffer (void):
    m_hBitmap (NULL)
  , m_hCanvas (NULL)
  , m_hBitmapP (NULL)
  {
    m_szBuffer.cx = 
    m_szBuffer.cy = 0;
  }

 ~CGdiBuffer (void)
  {
    Destroy ();
  }
  
  void Destroy (void)
  {
    if ( m_hBitmapP != NULL && m_hBitmap != NULL )
    {
      ::SelectObject ( m_hCanvas, m_hBitmapP );
    }

    if ( m_hBitmap != NULL )
    {
      ::DeleteObject (m_hBitmap);
    }

    if ( m_hCanvas != NULL )
    {
      ::DeleteDC (m_hCanvas);
    }
    m_hCanvas = NULL;
    m_hBitmap = NULL;
    m_hBitmapP= NULL;
  }

  BOOL 
  ResetCanvas (HWND hWnd, const SIZE &szBuffer)
  {
    ATLASSERT (::IsWindow (hWnd) != FALSE);

    ATLASSERT (szBuffer.cx > 0);
    ATLASSERT (szBuffer.cy > 0);

    BITMAPINFO bmpinfos;
 
    HDC hDC = ::GetDC (hWnd);
    ATLASSERT (hDC != NULL);

    RtlZeroMemory (& bmpinfos, sizeof (BITMAPINFO));

    bmpinfos.bmiHeader.biSize        =  sizeof (bmpinfos);
    bmpinfos.bmiHeader.biWidth       =  szBuffer.cx + 15 & -16;
    bmpinfos.bmiHeader.biHeight      = -szBuffer.cy;
    bmpinfos.bmiHeader.biPlanes      =  1;
    bmpinfos.bmiHeader.biBitCount    =  32;
    bmpinfos.bmiHeader.biCompression =  BI_RGB;

    Destroy ();

    m_hCanvas = CreateCompatibleDC (hDC);
    ATLASSERT (m_hCanvas != NULL);

    m_hBitmap = CreateDIBSection	(m_hCanvas, & bmpinfos
                               , DIB_RGB_COLORS, reinterpret_cast<void **> (& m_pBuffer), NULL, 0);
    ATLASSERT (m_hBitmap != NULL);

    m_hBitmapP = SelectObject (m_hCanvas, m_hBitmap);
    ATLASSERT (m_hBitmapP != NULL && m_hBitmapP != HGDI_ERROR);

    BOOL bResult = ReleaseDC (m_hWnd, hDC);
    ATLASSERT (bResult != FALSE);

    m_nPitch = bmpinfos.bmiHeader.biWidth * 4;
    m_hWnd = hWnd;

    m_szBuffer = szBuffer;
    return TRUE;
  }

  void BitBlt ( const POINT &ptDst
           , const POINT &ptSrc
             , const SIZE &size )
  {
    HDC hDC = ::GetDC (m_hWnd);
    ATLASSERT (hDC != NULL);

    ::BitBlt (hDC, ptDst.x, ptDst.y
                , size.cx, size.cy
              , m_hCanvas, ptSrc.x, ptSrc.y, SRCCOPY);

    BOOL bReleased = ::ReleaseDC (m_hWnd, hDC);
    ATLASSERT (bReleased != FALSE);
  }

  void StretchBlt ( const POINT &ptDst
           , const POINT &ptSrc
             , const SIZE &szSrc )
  {
    HDC hDC = ::GetDC (m_hWnd);
    ATLASSERT (hDC != NULL);

    RECT rcClient;
    GetClientRect (m_hWnd, & rcClient);

    ::StretchBlt ( hDC, ptDst.x, ptDst.y
                , rcClient.right - rcClient.left
                 , rcClient.bottom - rcClient.top
                , m_hCanvas, ptSrc.x, ptSrc.y
                 , szSrc.cx, szSrc.cy, SRCCOPY ); 

    BOOL bReleased = ::ReleaseDC (m_hWnd, hDC);
    ATLASSERT (bReleased != FALSE);
  }

  const CCanvas &GetCanvas (void) const 
  {
    return * this;
  }
  HWND m_hWnd;
  HBITMAP m_hBitmap;
  HDC m_hCanvas;
  HGDIOBJ m_hBitmapP;
};

struct CAudio
{
  struct CBuffer
  {
    PUINT8 m_pBuffer;
    PUINT8 m_pBufferP; // loop poll.
    PUINT32 m_pFreeMask;
    UINT_PTR m_nSetMask;
    UINT_PTR m_nResMask;
    UINT_PTR m_nFreeSamples;
  };

  CAudio (void): m_pXAudio2 (NULL)
                , m_pMasterVoice (NULL)
              , m_pSourceVoice (NULL)
               , m_nBuffers (0)
  {
    m_pFreeMask_ = new UINT8 [64 + 4];
    ATLASSERT (m_pFreeMask_ != nullptr);

    m_pFreeMask = reinterpret_cast<PUINT32>
            ( (reinterpret_cast<UINT_PTR> (m_pFreeMask_) + 63) & -64 );

    m_cBufferDesc.Flags = 0;
    m_cBufferDesc.LoopBegin = 0;
    m_cBufferDesc.LoopLength = 0;
    m_cBufferDesc.LoopCount = 0;
    m_cBufferDesc.PlayLength = 0;
    m_cBufferDesc.PlayBegin = 0;
  }

  class CAudioCallback : public IXAudio2VoiceCallback
  { // async audio callback.
  public:
    CAudioCallback (void){}
   ~CAudioCallback (void){}

    // Called just before this voice's processing pass begins.
    STDMETHOD_(void, OnVoiceProcessingPassStart) (THIS_ UINT32 BytesRequired) {}

    // Called just after this voice's processing pass ends.
    STDMETHOD_(void, OnVoiceProcessingPassEnd) (THIS) {}

    // Called when this voice has just finished playing a buffer stream
    // (as marked with the XAUDIO2_END_OF_STREAM flag on the last buffer).
    STDMETHOD_(void, OnStreamEnd) (THIS) {}

    // Called when this voice is about to start processing a new buffer.
    STDMETHOD_(void, OnBufferStart) (THIS_ void* pBufferContext) {}

    // Called when this voice has just finished processing a buffer.
    // The buffer can now be reused or destroyed.
    STDMETHOD_(void, OnBufferEnd) (THIS_ void* pBufferContext)
    {
      CBuffer *p = reinterpret_cast<CBuffer *> (pBufferContext);
      ATLASSERT (p != nullptr);

#if 0
      p->m_pFreeMask[0] |= p->m_nSetMask;
#else 
      LONG i;
      LONG j;
      LONG mask = p->m_nSetMask;

      j = * p->m_pFreeMask;

      do 
      {
        i = j;
        j = InterlockedCompareExchange ( p->m_pFreeMask, i | mask, i);
      } while (i != j);
#endif 
    }

    // Called when this voice has just reached the end position of a loop.
    STDMETHOD_(void, OnLoopEnd) (THIS_ void* pBufferContext) {}

    // Called in the event of a critical error during voice processing,
    // such as a failing xAPO or an error from the hardware XMA decoder.
    // The voice may have to be destroyed and re-created to recover from
    // the error.  The callback arguments report which buffer was being
    // processed when the error occurred, and its HRESULT code.
    STDMETHOD_(void, OnVoiceError) (THIS_ void* pBufferContext, HRESULT Error) {}
  };

 ~CAudio (void)
  {
    if ( m_pFreeMask_ != nullptr )
    {
      delete [] m_pFreeMask_;
    }
    
    if ( m_pSourceVoice != nullptr )
    {
      m_pSourceVoice->Stop ();
      m_pSourceVoice->FlushSourceBuffers ();
      m_pSourceVoice->DestroyVoice ();
    }
    
    if ( m_pMasterVoice != nullptr )
    {
      m_pMasterVoice->DestroyVoice ();
    }
    
    if ( m_pXAudio2 != nullptr )
    {
      m_pXAudio2->Release ();
    }
    m_pFreeMask_ = nullptr;
    m_pSourceVoice =  nullptr;
    m_pMasterVoice = nullptr;
    m_pXAudio2 = nullptr;
  }

  HRESULT 
  Release (void)
  {
    return S_OK;
  }

  HRESULT 
  Init (void)
  {
    HRESULT hResult = S_OK;

    if ( FAILED (hResult = XAudio2Create ( & m_pXAudio2, 0, XAUDIO2_DEFAULT_PROCESSOR ) ) )
    {
      return hResult;
    }
    
    if ( FAILED (hResult = m_pXAudio2->CreateMasteringVoice ( & m_pMasterVoice ) ) )
    {
      return hResult;
    }
    return hResult;
  }

  HRESULT
  ResetPCM ( UINT nSamplesPerSec, WORD wFormatTag, WORD wSampleBits)
  {
    HRESULT hResult;

    ATLASSERT (wFormatTag == WAVE_FORMAT_IEEE_FLOAT || wFormatTag == WAVE_FORMAT_PCM);
    ATLASSERT (wFormatTag != WAVE_FORMAT_IEEE_FLOAT || wSampleBits == 32);
    ATLASSERT (wFormatTag != WAVE_FORMAT_PCM || (wSampleBits == 8 || wSampleBits == 16));

    if ( m_pSourceVoice != NULL) 
    {
      m_pSourceVoice->Stop ();
      m_pSourceVoice->FlushSourceBuffers ();
      m_pSourceVoice->DestroyVoice ();
      m_pSourceVoice = NULL;
    }
    m_cWaveInfos.cbSize = sizeof (WAVEFORMATEX);
    m_cWaveInfos.nChannels = 2; // always 2
    m_cWaveInfos.wBitsPerSample = wSampleBits;
    m_cWaveInfos.nBlockAlign = (m_cWaveInfos.nChannels * m_cWaveInfos.wBitsPerSample) / 8;
    m_cWaveInfos.nSamplesPerSec = nSamplesPerSec;
    m_cWaveInfos.wFormatTag = wFormatTag;
    m_cWaveInfos.nAvgBytesPerSec = m_cWaveInfos.nBlockAlign * m_cWaveInfos.nSamplesPerSec;

    if( FAILED (hResult = m_pXAudio2->CreateSourceVoice ( & m_pSourceVoice, & m_cWaveInfos, 0, 2.0f, & m_callback ) ) )
    {
      m_pSourceVoice = NULL;
      return hResult;
    }
    return hResult;
  }

  IR_FORCEINLINE
  HRESULT 
  SubmitSampleSigned16 (int16_t left, int16_t right)
  {
    HRESULT hResult = S_OK;

    CAudio::CBuffer *pCurrent = m_pCurrent;
    ATLASSERT (pCurrent != nullptr);

    reinterpret_cast<uint16_t *> (& pCurrent->m_pBufferP[0] )[0] = left;
    reinterpret_cast<uint16_t *> (& pCurrent->m_pBufferP[0] )[1] = right;

    pCurrent->m_pBufferP += 4;

    if ( --pCurrent->m_nFreeSamples == 0 )
    { // post sound buffer.
      volatile UINT32 &fatomic_mask = m_pFreeMask[0];

#if 0
      fatomic_mask &= pCurrent->m_nResMask; // clear free mask 
#else 
      LONG i;
      LONG j;
      LONG mask = pCurrent->m_nResMask;

      j = fatomic_mask;

      do 
      {
        i = j;
        j = InterlockedCompareExchange ( & fatomic_mask, i & mask, i);
      } while (i != j);
#endif 
      m_cBufferDesc.pAudioData = reinterpret_cast<const BYTE *> (pCurrent->m_pBuffer);
      m_cBufferDesc.pContext = pCurrent;

      hResult = m_pSourceVoice->SubmitSourceBuffer (& m_cBufferDesc);

      while ( fatomic_mask == 0 )
      { // get free buffer.
        _mm_pause ();
      }
      pCurrent = m_pSoundBuffers[_tzcnt_u32 (fatomic_mask)];
      pCurrent->m_pBufferP = pCurrent ->m_pBuffer;
      pCurrent->m_nFreeSamples = m_nBufferSamples;

      m_pCurrent = pCurrent;
    }
    return hResult;
  }

  BOOL
  ResetBuffers ( UINT nBuffers )
  {
    ATLASSERT (nBuffers >= 4 && nBuffers <= 32);
    ATLASSERT (m_pSourceVoice != nullptr);

    for (UINT n = 0; n != m_nBuffers; n++)
    { // release previous 
      CBuffer *buffer = m_pSoundBuffers[n];
      ATLASSERT (buffer != nullptr);

      ::free (buffer);
    }
    m_pSoundBuffers = new CBuffer *[nBuffers];
    ATLASSERT (m_pSoundBuffers != nullptr);

    m_nBufferSamples = m_cWaveInfos.nSamplesPerSec / 60;
    m_nBufferBytes = m_cWaveInfos.nBlockAlign * m_nBufferSamples;
    
    m_cBufferDesc.AudioBytes = m_nBufferBytes; // update post member.

    for (UINT n = 0; n != nBuffers; n++)
    { // create new buffer
      CBuffer *pBuffer = new CBuffer;
      ATLASSERT (pBuffer != nullptr);

      pBuffer->m_pBuffer = new BYTE[m_nBufferBytes];
      pBuffer->m_nSetMask = 1 << n;
      pBuffer->m_nResMask =~pBuffer->m_nSetMask;
      pBuffer->m_pFreeMask= m_pFreeMask;

      ATLASSERT (pBuffer->m_pBuffer != nullptr);

      m_pSoundBuffers[n] = pBuffer;
    }
    m_pFreeMask[0] = 1 << nBuffers;
    m_pFreeMask[0]-= 1;

    m_pCurrent = m_pSoundBuffers[0];
    m_pCurrent->m_nFreeSamples = m_nBufferSamples;
    m_pCurrent->m_pBufferP = m_pCurrent->m_pBuffer;

    m_pFreeMask[0] &= ~1;

    m_nBuffers = nBuffers;
    return TRUE;
  }

  HRESULT 
  Play (void)
  {
    ATLASSERT (m_pSourceVoice != nullptr);

    return m_pSourceVoice->Start ();
  }

  IXAudio2 *m_pXAudio2;
  IXAudio2MasteringVoice *m_pMasterVoice;
  IXAudio2SourceVoice *m_pSourceVoice;

  CBuffer **m_pSoundBuffers;
  CBuffer *m_pCurrent; // current render sound buffer.

  CAudioCallback m_callback;

  UINT32 m_nBufferBytes;
  UINT32 m_nBufferSamples;
  UINT32 m_nBuffers; // 1/60s buffer

  PUINT8 m_pFreeMask_; 
  PUINT32 m_pFreeMask; // align, atomic read/write

  WAVEFORMATEX m_cWaveInfos;

  XAUDIO2_BUFFER m_cBufferDesc;
};

struct Gb : public gb<Gb>
{
  Gb (void) : m_nKeyStateMask (0)
  {
    
  }

  IR_FORCEINLINE
  void os_snd_synthesis ( uint32_t slch0, uint32_t slch1
                      , uint32_t slch2, uint32_t slch3 
                       , uint32_t srch0, uint32_t srch1
                      , uint32_t srch2, uint32_t srch3 )
  {
    slch0 *= 0x1111; // amplify the volume (0~0xffff)
    slch1 *= 0x1111; // amplify the volume (0~0xffff)
    slch2 *= 0x1111; // amplify the volume (0~0xffff)
    slch3 *= 0x1111; // amplify the volume (0~0xffff)
    srch0 *= 0x1111; // amplify the volume (0~0xffff)
    srch1 *= 0x1111; // amplify the volume (0~0xffff)
    srch2 *= 0x1111; // amplify the volume (0~0xffff)
    srch3 *= 0x1111; // amplify the volume (0~0xffff)

    uint32_t slch = slch0 + slch1
                 + slch2 + slch3; // mix left channel 
    uint32_t srch = srch0 + srch1
                 + srch2 + srch3; // mix right channel

    uint16_t slch16 = slch >> 2; // clamp to [0, 0xffff]
    uint16_t srch16 = srch >> 2; // clamp to [0, 0xffff]
             slch16-= 0x8000; // unsigned 16bit pcm convert to signed 16bit pcm
             srch16-= 0x8000; // unsigned 16bit pcm convert to signed 16bit pcm

    m_hAudio.SubmitSampleSigned16 ( slch16, srch16 );
  }

  int os_vid_lock (uint8_t *&lockptr, uintptr_t &pitch)
  {
    lockptr = reinterpret_cast<uint8_t *> (m_hCanvas.m_pBuffer);
    pitch = m_hCanvas.m_nPitch;
    return 0;
  }

  int os_vid_unlock (void)
  {
    return 0;
  }

  int os_vid_islock (void)
  {
    return 1;
  }

  int os_vid_flush (void)
  {
    m_hCanvas.StretchBlt (  CPoint (0, 0), CPoint ( 8, 0 ), CSize (160, 144 ) );
    return 0;
  }

  int os_timing_sync (void)
  {
    return 0;
  }

  IR_FORCEINLINE
  int os_joyp_update (uint8_t *key)
  {
    * key = m_nKeyStateMask;
    return 0;
  }

  void OnIdle (void)
  {
    gb::frame ();
    
    // Sleep (1);
  }

  BOOL 
  OnMessage (LRESULT &lResult, HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
  {
    switch ( msg )
    {
  case WM_KEYDOWN:
    switch ( wParam )
    {
    case 'D': m_nKeyStateMask |= 1; break;
    case 'A': m_nKeyStateMask |= 2; break;
    case 'W': m_nKeyStateMask |= 4; break;
    case 'S': m_nKeyStateMask |= 8; break;
    case 'K': m_nKeyStateMask |= 16; break;
    case 'J': m_nKeyStateMask |= 32; break;
    case ' ': m_nKeyStateMask |= 64; break;
    case VK_RETURN: m_nKeyStateMask |= 128; break;
    default:
      break;
    }
    break;
  case WM_KEYUP:
    switch ( wParam )
    {
    case 'D': m_nKeyStateMask &= ~1; break;
    case 'A': m_nKeyStateMask &= ~2; break;
    case 'W': m_nKeyStateMask &= ~4; break;
    case 'S': m_nKeyStateMask &= ~8; break;
    case 'K': m_nKeyStateMask &= ~16; break;
    case 'J': m_nKeyStateMask &= ~32; break;
    case ' ': m_nKeyStateMask &= ~64; break;
    case VK_RETURN: m_nKeyStateMask &= ~128; break;
    default:
      break;
    }
  default:
    return FALSE;
    }


    return FALSE;
  }

  int OpenRom (LPCWSTR pstrRomPath, UINT32 nForceDmgMask = 0)
  {
    HANDLE hFile = CreateFileW ( pstrRomPath, GENERIC_READ, FILE_SHARE_READ, NULL
                           , OPEN_EXISTING, FILE_FLAG_SEQUENTIAL_SCAN, NULL);
    DWORD nBytes = GetFileSize (hFile, NULL);
    HANDLE hMap = CreateFileMappingW ( hFile, NULL, PAGE_READONLY, 0, nBytes, NULL );

    LPVOID lpData = MapViewOfFile ( hMap, FILE_MAP_READ, 0, 0, 0 );

    int error = 0;

    if ( hFile == INVALID_HANDLE_VALUE )
    {
      if ( GetLastError () == ERROR_ACCESS_DENIED )
      {
        error = 1;
        goto cleanup;
      }
    }

    if ( nBytes == INVALID_FILE_SIZE )
    {
      error = 2;
      goto cleanup;
    }
    else if ( hMap == 0 )
    {
      error = 3;
      goto cleanup;
    }

    if ( hMap == NULL )
    { // mmap failed
      error = 4;
      goto cleanup;
    }

    if ( GetLastError () == ERROR_ALREADY_EXISTS )
    { // object exist 
      error = 5;
      goto cleanup;
    }

    if ( lpData == NULL )
    { // unable to establish correct mapping
      error = 6;
      goto cleanup;
    } 
    error = gb::cart::load ( reinterpret_cast<uint8_t *> (lpData), nBytes, nForceDmgMask);

    if ( error != 0 )
    {
      error += 128;
    }
cleanup:
    if (hMap != NULL)
    {
      CloseHandle (hMap);
    }

    if (hFile != INVALID_HANDLE_VALUE)
    {
      CloseHandle (hFile);
    }
    return error;
  }
  CGdiBuffer m_hCanvas;
  CAudio m_hAudio;

  BYTE m_nKeyStateMask;

  HWND m_hWnd;
};

Gb gb;

LRESULT 
CALLBACK WndProc (HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
  LRESULT lResult = 0;

  if ( gb.OnMessage ( lResult, hWnd, message, wParam, lParam ) != FALSE )
  {
    return lResult;
  }
  return DefWindowProc (hWnd, message, wParam, lParam);
#if 0
  switch (message)
  {
  case WM_DESTROY:
    PostQuitMessage(0);
    return 0;
  case WM_KEYDOWN:
    switch ( wParam )
    {
    case 'D': g_joypad_key |= 1; break;
    case 'A': g_joypad_key |= 2; break;
    case 'W': g_joypad_key |= 4; break;
    case 'S': g_joypad_key |= 8; break;
    case 'K': g_joypad_key |= 16; break;
    case 'J': g_joypad_key |= 32; break;
    case ' ': g_joypad_key |= 64; break;
    case VK_RETURN: g_joypad_key |= 128; break;
    default:
      break;
    }
    break;
  case WM_KEYUP:
    switch ( wParam )
    {
    case 'D': g_joypad_key &= ~1; break;
    case 'A': g_joypad_key &= ~2; break;
    case 'W': g_joypad_key &= ~4; break;
    case 'S': g_joypad_key &= ~8; break;
    case 'K': g_joypad_key &= ~16; break;
    case 'J': g_joypad_key &= ~32; break;
    case ' ': g_joypad_key &= ~64; break;
    case VK_RETURN: g_joypad_key &= ~128; break;
    default:
      break;
    }
    break;
  default:
    break;
  }
  return DefWindowProc (hWnd, message, wParam, lParam);
#endif 
}





int 
WINAPI WinMain (HINSTANCE hInstance, HINSTANCE hPrevInstance, PSTR szCmdLine, int iCmdShow)
{
  HWND hWnd;
  MSG msg;

  WNDCLASS wndclass;

  wndclass.style = CS_HREDRAW | CS_VREDRAW;
  wndclass.lpfnWndProc = WndProc;
  wndclass.cbClsExtra = 0;
  wndclass.cbWndExtra = 0;
  wndclass.hInstance = hInstance;
  wndclass.hIcon = LoadIcon (NULL, IDI_APPLICATION);
  wndclass.hCursor = LoadCursor (NULL, IDC_ARROW);
  wndclass.hbrBackground = reinterpret_cast<HBRUSH> ( GetStockObject (WHITE_BRUSH) );
  wndclass.lpszMenuName = NULL;
  wndclass.lpszClassName = _T ("gb");

  if (!RegisterClass(&wndclass))
  {
      MessageBox(NULL, TEXT("This program requires Windows NT!"),
          _T ("gb"), MB_ICONERROR);
      return 0;
  }

HRESULT hr;
hr = CoInitializeEx( nullptr, COINIT_MULTITHREADED );
if (FAILED(hr))
    return hr;

  hWnd = CreateWindow (_T ("gb"), // window class name
      TEXT("VirtualZ80 "), // window caption
      WS_OVERLAPPEDWINDOW, // window style
      CW_USEDEFAULT, // initial x position
      CW_USEDEFAULT, // initial y position
      480, // initial x size
      320, // initial y size
      NULL, // parent window handle
      NULL, // window menu handle
      hInstance, // program instance handle
      NULL); // creation parameters
  ShowWindow (hWnd, iCmdShow);
  UpdateWindow (hWnd);

  gb.m_hWnd = hWnd;
  gb.m_hCanvas.ResetCanvas (hWnd, CSize (256, 256));
  gb.m_hAudio.Init ();
  gb.m_hAudio.ResetPCM (44100, WAVE_FORMAT_PCM, 16);
  gb.m_hAudio.ResetBuffers (8);
  gb.m_hAudio.Play ();
  gb.OpenRom (ROM_PATH, 0);
  gb.reset ();
  gb.resume ();

  while (TRUE)
  {
    if (PeekMessage (& msg, NULL, 0, 0, PM_REMOVE) != FALSE)
    {
      if (msg.message == WM_QUIT)
      {
        break ;
      }
      TranslateMessage (& msg);
      DispatchMessage (& msg);
    }
    else
    {//printf ("5555555555\n");
      gb.OnIdle ();
    }
  }
  return msg.wParam;
}

int main (void)
{
  HINSTANCE hInstance = GetModuleHandle (NULL);
  HINSTANCE hPrevInstance = NULL;

  WinMain (hInstance, hPrevInstance, NULL, SW_SHOWNORMAL);
  return 0;
}

uint32_t dbg_flags = 0;
