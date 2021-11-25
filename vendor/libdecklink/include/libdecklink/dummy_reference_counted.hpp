//
// Created by tibo on 18/01/18.
//

#pragma once

#define DUMMY_REF_COUNT_METHODS \
    virtual HRESULT QueryInterface(REFIID, LPVOID*) override {        \
        return E_FAIL;                                                \
    }                                                                 \
                                                                      \
    virtual ULONG AddRef() override {                                 \
        return 0;                                                     \
    }                                                                 \
                                                                      \
    virtual ULONG Release() override {                                \
        return 0;                                                     \
    }                                                                 \

