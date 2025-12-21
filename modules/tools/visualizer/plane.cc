/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/tools/visualizer/plane.h"

std::shared_ptr<Texture> Plane::NullTextureObj;

Plane::Plane(const std::shared_ptr<Texture>& t)
    : RenderableObject(6, 4), texture_id_(0), texture_(t) {}

bool Plane::FillVertexBuffer(GLfloat* pBuffer) {
  if (texture_ == nullptr || !texture_->isDirty()) {
    return false;
  }
  glGenTextures(1, &texture_id_);

  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  // Clamp to edge for ES/core profile compatibility
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // Prefer base internal formats for ES/core compatibility and match channels
  GLenum fmt = texture_->texture_format();
  GLint internal_fmt = (fmt == GL_RGBA) ? GL_RGBA : GL_RGB;
  glTexImage2D(GL_TEXTURE_2D, 0, internal_fmt, texture_->width(),
               texture_->height(), 0, fmt, GL_UNSIGNED_BYTE,
               texture_->data());

  glBindTexture(GL_TEXTURE_2D, 0);

  texture_->removeDirty();

  // Two CCW triangles forming a full-screen quad
  // Triangle 0: BL, BR, TR
  pBuffer[0] = -1.0f;   // BL.x
  pBuffer[1] = -1.0f;   // BL.y
  pBuffer[2] = 0.0f;    // BL.u
  pBuffer[3] = 0.0f;    // BL.v

  pBuffer[4] = 1.0f;    // BR.x
  pBuffer[5] = -1.0f;   // BR.y
  pBuffer[6] = 1.0f;    // BR.u
  pBuffer[7] = 0.0f;    // BR.v

  pBuffer[8] = 1.0f;    // TR.x
  pBuffer[9] = 1.0f;    // TR.y
  pBuffer[10] = 1.0f;   // TR.u
  pBuffer[11] = 1.0f;   // TR.v

  // Triangle 1: BL, TR, TL
  pBuffer[12] = -1.0f;  // BL.x
  pBuffer[13] = -1.0f;  // BL.y
  pBuffer[14] = 0.0f;   // BL.u
  pBuffer[15] = 0.0f;   // BL.v

  pBuffer[16] = 1.0f;   // TR.x
  pBuffer[17] = 1.0f;   // TR.y
  pBuffer[18] = 1.0f;   // TR.u
  pBuffer[19] = 1.0f;   // TR.v

  pBuffer[20] = -1.0f;  // TL.x
  pBuffer[21] = 1.0f;   // TL.y
  pBuffer[22] = 0.0f;   // TL.u
  pBuffer[23] = 1.0f;   // TL.v

  return true;
}

void Plane::SetupAllAttrPointer(void) {
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(
      0, 2, GL_FLOAT, GL_FALSE,
      static_cast<int>(sizeof(GLfloat)) * vertex_element_count(), 0);

  glEnableVertexAttribArray(1);
  glVertexAttribPointer(
      1, 2, GL_FLOAT, GL_FALSE,
      static_cast<int>(sizeof(GLfloat)) * vertex_element_count(),
      reinterpret_cast<void*>(sizeof(GLfloat) * 2));
}

void Plane::Draw(void) {
  if (texture_->data()) {
    if (texture_->isSizeChanged()) {
      glGenTextures(1, &texture_id_);

      glBindTexture(GL_TEXTURE_2D, texture_id_);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

      GLenum fmt = texture_->texture_format();
      GLint internal_fmt = (fmt == GL_RGBA) ? GL_RGBA : GL_RGB;
      glTexImage2D(GL_TEXTURE_2D, 0, internal_fmt, texture_->width(),
                   texture_->height(), 0, fmt, GL_UNSIGNED_BYTE,
                   texture_->data());

      glBindTexture(GL_TEXTURE_2D, 0);

      texture_->removeDirty();
    } else if (texture_->isDirty()) {
      glBindTexture(GL_TEXTURE_2D, texture_id_);
      glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture_->width(),
                      texture_->height(), texture_->texture_format(),
                      GL_UNSIGNED_BYTE, texture_->data());
      glBindTexture(GL_TEXTURE_2D, 0);
      texture_->removeDirty();
    }
  }

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  // Avoid culling artifacts on some platforms by temporarily disabling
  // face culling while drawing the screen-aligned quad.
  GLboolean cull_enabled = glIsEnabled(GL_CULL_FACE);
  if (cull_enabled) {
    glDisable(GL_CULL_FACE);
  }
  RenderableObject::Draw();
  if (cull_enabled) {
    glEnable(GL_CULL_FACE);
  }
  glBindTexture(GL_TEXTURE_2D, 0);
}

void Plane::SetupExtraUniforms(void) {
  // Ensure sampler uses texture unit 0
  if (shader_program_) {
    shader_program_->setUniformValue("u_texture", 0);
    // Backward compatibility: if shader still uses 'texture' as uniform name
    // (older shader), try to set it too.
    shader_program_->setUniformValue("texture", 0);
  }
}

