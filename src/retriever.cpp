/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/filesystem.hpp>
#include "resource_retriever/retriever.h"

#include <string.h>
#include <console_bridge/console.h>

#include <curl/curl.h>

namespace resource_retriever
{

class CURLStaticInit
{
public:
  CURLStaticInit()
  : initialized_(false)
  {
    CURLcode ret = curl_global_init(CURL_GLOBAL_ALL);
    if (ret != 0)
    {
      CONSOLE_BRIDGE_logError ("Error initializing libcurl! retcode = %d", ret);
    }
    else
    {
      initialized_ = true;
    }
  }

  ~CURLStaticInit()
  {
    if (initialized_)
    {
      curl_global_cleanup();
    }
  }

  bool initialized_;
};
static CURLStaticInit g_curl_init;

Retriever::Retriever()
{
  curl_handle_ = curl_easy_init();
}

Retriever::~Retriever()
{
  if (curl_handle_)
  {
    curl_easy_cleanup(curl_handle_);
  }
}

struct MemoryBuffer
{
  std::vector<uint8_t> v;
};

size_t curlWriteFunc(void* buffer, size_t size, size_t nmemb, void* userp)
{
  MemoryBuffer* membuf = (MemoryBuffer*)userp;

  size_t prev_size = membuf->v.size();
  membuf->v.resize(prev_size + size * nmemb);
  memcpy(&membuf->v[prev_size], buffer, size * nmemb);

  return size * nmemb;
}

/**
 * @brief      Retrieve the path of the file whose path is given in an url-format.
 *             Currently convert from the folliwing patterns : package:// or file://
 *
 * @param[in]  string          The path given in the url-format
 * @param[in]  package_dirs    A list of packages directories where to search for files
 *                             if its pattern starts with package://
 *
 * @return     The path to the file (can be a relative or absolute path)
 */
  inline std::string retrieveResourcePath(const std::string & string,
					const std::vector<std::string> & package_dirs) throw (std::invalid_argument)
{

  namespace bf = boost::filesystem;
  std::string result_path;

  const std::string separator("://");
  const std::size_t pos_separator = string.find(separator);

  if (pos_separator != std::string::npos)
    {
      std::string scheme = string.substr(0, pos_separator);
      std::string path = string.substr(pos_separator+3, std::string::npos);

      if(scheme == "package")
	{
	  // if exists p1/string, path = p1/string,
	  // else if exists p2/string, path = p2/string
	  // else return an empty string that may provoke an error in loadPolyhedronFromResource()

	  // concatenate package_path with filename
	  std::ostringstream errorMsg;
	  errorMsg << "None of the following files was found: ";
	  for (std::size_t i = 0; i < package_dirs.size(); ++i)
	    {
	      std::string candidate (package_dirs[i] + "/" + path);
	      if ( bf::exists( bf::path(candidate)))
		{
		  result_path = std::string ("file://") + candidate;
		  return result_path;
		}
	      else {
		errorMsg << candidate;
		if (i+1 < package_dirs.size())
		  errorMsg << ", ";
		else
		  errorMsg << ".";
	      }
	    }
	  // If file not found, throw an error
	  throw std::invalid_argument (errorMsg.str ().c_str ());
	}
      else if (scheme == "file")
	{
	  result_path = path;
	}
      else
	{
	  const std::string exception_message ("Schemes of form" + scheme + "are not handled");
	  throw std::invalid_argument(exception_message);
	}
    }
  else // return the entry string
    {
      result_path = string;
      assert(false && "the path does not respect the pattern package:// or file://");
    }

  return result_path;
}

  /**
   * @brief      Parse an environment variable if exists and extract paths according to the delimiter.
   *
   * @param[in]  env_var_name The name of the environment variable.
   * @param[in]  delimiter The delimiter between two consecutive paths.
   *
   * @return The vector of paths extracted from the environment variable value.
   */
  inline std::vector<std::string> extractPathFromEnvVar(const std::string & env_var_name, const std::string & delimiter = ":")
  {
    const char * env_var_value = std::getenv(env_var_name.c_str());
    std::vector<std::string> env_var_paths;

    if (env_var_value != NULL)
    {
      std::string policyStr (env_var_value);
      // Add a separator at the end so that last path is also retrieved
      policyStr += std::string (":");
      size_t lastOffset = 0;

      while(true)
      {
        size_t offset = policyStr.find_first_of(delimiter, lastOffset);
        if (offset < policyStr.size())
          env_var_paths.push_back(policyStr.substr(lastOffset, offset - lastOffset));
        if (offset == std::string::npos)
          break;
        else
          lastOffset = offset + 1; // add one to skip the delimiter
      }
    }

    return env_var_paths;
  }

/**
 * @brief      Parse the environment variable ROS_PACKAGE_PATH and extract paths
 *
 * @return     The vector of paths extracted from the environment variable ROS_PACKAGE_PATH
 */
inline std::vector<std::string> rosPaths()
{
  return extractPathFromEnvVar("ROS_PACKAGE_PATH");
}


MemoryResource Retriever::get(const std::string& url)
{
  std::vector <std::string> paths (rosPaths ());
  std::string mod_url = retrieveResourcePath (url, paths);

  curl_easy_setopt(curl_handle_, CURLOPT_URL, mod_url.c_str());
  curl_easy_setopt(curl_handle_, CURLOPT_WRITEFUNCTION, curlWriteFunc);

  char error_buffer[CURL_ERROR_SIZE];
  curl_easy_setopt(curl_handle_, CURLOPT_ERRORBUFFER , error_buffer);

  MemoryResource res;
  MemoryBuffer buf;
  curl_easy_setopt(curl_handle_, CURLOPT_WRITEDATA, &buf);

  CURLcode ret = curl_easy_perform(curl_handle_);
  if (ret != 0)
  {
    throw Exception(mod_url, error_buffer);
  }
  else if (!buf.v.empty())
  {
    res.size = (unsigned int) buf.v.size();
    res.data.reset(new uint8_t[res.size]);
    memcpy(res.data.get(), &buf.v[0], res.size);
  }

  return res;
}

}
